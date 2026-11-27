"""Replace a local driver container, preserving Docker config, with rollback.

Run on the robot: python3 local_image.py IMAGE BACKUP_DIRECTORY
Restore later: python3 local_image.py --rollback BACKUP_DIRECTORY
"""
import copy
import http.client
import json
import socket
import sys
import time
import urllib.request
from pathlib import Path

NAME = 'embodied-unixai-panther'


class Docker(http.client.HTTPConnection):
    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(90)
        self.sock.connect('/var/run/docker.sock')


def api(method, path, data=None, allowed=(200, 201, 204, 304)):
    conn = Docker('localhost', timeout=90)
    conn.request(method, '/v1.44' + path, json.dumps(data) if data is not None else None,
                 {'Content-Type': 'application/json'})
    response = conn.getresponse()
    raw = response.read()
    conn.close()
    if response.status not in allowed:
        raise RuntimeError(f'{method} {path}: {response.status} {raw.decode()}')
    return json.loads(raw) if raw else None


def replace(payload):
    api('POST', f'/containers/{NAME}/stop?t=30', allowed=(204, 304, 404))
    api('DELETE', f'/containers/{NAME}', allowed=(204, 404))
    api('POST', f'/containers/create?name={NAME}', payload)
    api('POST', f'/containers/{NAME}/start')


def healthy(require_card):
    for _ in range(30):
        try:
            state = api('GET', f'/containers/{NAME}/json')['State']
            assert state['Running']
            request = urllib.request.Request('http://127.0.0.1:15792/mcp',
                data=json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': 'tools/list', 'params': {}}).encode(),
                headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(request, timeout=3) as response:
                tools = json.load(response)['result']['tools']
            assert not require_card or any(t['name'] == 'arm_point' for t in tools)
            return
        except Exception:
            time.sleep(2)
    raise RuntimeError('MCP health check failed')


def main():
    target, directory = sys.argv[1:]
    backup = Path(directory)
    if target == '--rollback':
        replace(json.loads((backup / 'rollback.json').read_text()))
        healthy(False)
        print('ROLLBACK_OK', backup)
        return
    backup.mkdir(mode=0o700, parents=False, exist_ok=False)
    current = api('GET', f'/containers/{NAME}/json')
    (backup / 'inspect.json').write_text(json.dumps(current, indent=2))
    original = copy.deepcopy(current['Config'])
    original['HostConfig'] = current['HostConfig']
    original['Image'] = current['Image']
    (backup / 'rollback.json').write_text(json.dumps(original, indent=2))
    updated = copy.deepcopy(original)
    updated['Image'] = api('GET', f'/images/{target}/json')['Id']
    (backup / 'new.json').write_text(json.dumps(updated, indent=2))
    try:
        replace(updated)
        healthy(True)
        deployed = api('GET', f'/containers/{NAME}/json')
        assert deployed['Config']['Env'] == current['Config']['Env']
        assert deployed['HostConfig']['Binds'] == current['HostConfig']['Binds']
        assert deployed['Image'] == updated['Image']
        print('DEPLOY_OK', deployed['Image'], 'backup', backup)
    except Exception:
        replace(original)
        healthy(False)
        print('Deployment failed; original image restored', flush=True)
        raise


if __name__ == '__main__':
    main()
