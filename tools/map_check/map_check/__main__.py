"""python -m map_check [--port N]"""
import argparse
import os

import uvicorn

from .config import Settings
from .server import create_app


def main(argv=None):
    ap = argparse.ArgumentParser(prog='map_check', description=__doc__)
    ap.add_argument('--port', type=int, help='port to serve on (default 9081)')
    args = ap.parse_args(argv)
    env = dict(os.environ)
    if args.port is not None:
        env['MAP_CHECK_PORT'] = str(args.port)
    settings = Settings.from_env(env)
    uvicorn.run(create_app(settings), host=settings.host, port=settings.port)


if __name__ == '__main__':
    main()
