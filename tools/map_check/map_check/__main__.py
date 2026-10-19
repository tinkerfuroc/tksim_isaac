"""python -m map_check [--port N]"""
import argparse
import os
import sys

from .config import Settings
from .server import LOOPBACK_HOSTS, create_app, serve


def main(argv=None):
    ap = argparse.ArgumentParser(prog='map_check', description=__doc__)
    ap.add_argument('--port', type=int, help='port to serve on (default 9081)')
    args = ap.parse_args(argv)
    env = dict(os.environ)
    if args.port is not None:
        env['MAP_CHECK_PORT'] = str(args.port)
    settings = Settings.from_env(env)
    if settings.host not in LOOPBACK_HOSTS:
        sys.exit('Map Check binds to loopback only; use an ssh tunnel for remote access '
                 f'(got host {settings.host!r}).')
    serve(create_app(settings), settings.host, settings.port)


if __name__ == '__main__':
    main()
