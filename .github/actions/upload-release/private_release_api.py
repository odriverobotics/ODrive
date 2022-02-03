
import hashlib
import os

from odrive.api_client import ApiClient
from odrive.crypto import b64encode

class PrivateReleaseApi():
    BASE_URL = '/releases'

    @staticmethod
    def get_content_key(path: str, commit_hash: str):
        commit_hash_bytes = bytes.fromhex(commit_hash)

        def _get_file_names(path: str, prefix: list):
            with os.scandir(os.path.join(path, *prefix)) as it:
                for entry in it:
                    if entry.is_file():
                        yield os.path.join(*prefix, entry.name)
                    else:
                        yield from _get_file_names(path, prefix + [entry.name])
        filenames = sorted(_get_file_names(path, []))

        dir_hasher = hashlib.sha256()
        
        for filename in filenames:
            with open(os.path.join(path, filename), 'rb') as fp:
                content = fp.read()

            # Calculate commit-invariant hash of the file content
            # This means that if a new compile differs only by embedded commit hash,
            # it is considered equal.
            patched_content = content.replace(commit_hash_bytes, bytes(len(commit_hash_bytes)))
            file_hasher = hashlib.sha256()
            file_hasher.update(patched_content)

            dir_hasher.update(filename.encode('utf-8'))
            dir_hasher.update(file_hasher.digest())

        return dir_hasher.digest()

    def __init__(self, api_client: 'ApiClient'):
        self._api_client = api_client

    async def get_manifest(self, release_type: str, content_key: bytes):
        outputs = await self._api_client.call('GET', PrivateReleaseApi.BASE_URL + '/' + release_type + '/manifest', inputs={
            'content_key': b64encode(content_key),
        })
        return outputs

    async def register_content(self, release_type: str, commit_hash: str, content_key: bytes, **qualifiers):
        args = {
            'commit_hash': commit_hash,
            'content_key': b64encode(content_key),
            **qualifiers
        }
        outputs = await self._api_client.call('PUT', PrivateReleaseApi.BASE_URL + '/' + release_type + '/content', inputs=args)
        return outputs['created']

    async def register_commit(self, release_type: str, channel: str, commit_hash: str):
        outputs = await self._api_client.call('PUT', PrivateReleaseApi.BASE_URL + '/' + release_type + '/commit', inputs={
            'commit_hash': commit_hash,
            'channel': channel
        })
        return outputs['published']

    async def refresh_routes(self, release_type: str):
        await self._api_client.call('PUT', PrivateReleaseApi.BASE_URL + '/' + release_type + '/refresh-routes')
