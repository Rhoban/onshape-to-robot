import os, sys, json, io, builtins, contextlib, argparse
from urllib.parse import urlsplit, parse_qsl
from onshape_to_robot.onshape_api.onshape import Onshape


@contextlib.contextmanager
def _mem_creds(fake_path: str, payload: dict):
    real_open, real_exists, real_isfile = builtins.open, os.path.exists, os.path.isfile
    blob = json.dumps(payload)

    def open_hook(p, *a, **kw):
        mode = (a[0] if a else kw.get("mode", "r"))
        return io.StringIO(
            blob) if p == fake_path and "r" in mode else real_open(
                p, *a, **kw)

    def exists_hook(p):
        return True if p == fake_path else real_exists(p)

    def isfile_hook(p):
        return True if p == fake_path else real_isfile(p)

    builtins.open, os.path.exists, os.path.isfile = open_hook, exists_hook, isfile_hook
    try:
        yield
    finally:
        builtins.open, os.path.exists, os.path.isfile = real_open, real_exists, real_isfile


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--stack",
                        default=os.getenv("ONSHAPE_STACK",
                                          "https://cad.onshape.com"))
    parser.add_argument("--path", default="/api/documents")
    parser.add_argument("--query", default="limit=1")
    parser.add_argument("--method", default="GET", choices=["GET", "HEAD"])
    parser.add_argument("--access-key",
                        default=os.getenv("ONSHAPE_ACCESS_KEY"))
    parser.add_argument("--secret-key",
                        default=os.getenv("ONSHAPE_SECRET_KEY"))
    args, _ = parser.parse_known_args()
    if not args.access_key or not args.secret_key:
        print("ERROR: set --access-key/--secret-key or ONSHAPE_* env.")
        sys.exit(2)

    parts = urlsplit(args.path)
    path_only = parts.path or "/"
    q = dict(parse_qsl(parts.query))
    if args.query: q.update(dict(parse_qsl(args.query)))

    fake_creds = ":mem:onshape_creds:"
    payload = {
        "onshape_access_key": args.access_key,
        "onshape_secret_key": args.secret_key,
        "access_key": args.access_key,
        "secret_key": args.secret_key,
    }

    with _mem_creds(fake_creds, payload):
        client = Onshape(args.stack, creds=fake_creds, logging=False)
        res = client.request(args.method,
                             path_only,
                             query=q,
                             headers={},
                             body=None,
                             base_url=None)

    hdr = dict(res.headers)
    print("=== Onshape API rate-limit check ===")
    print(f"Status                 : {res.status_code}")
    if "Date" in hdr: print(f"Date (server)          : {hdr['Date']}")
    if "X-Api-Version" in hdr:
        print(f"X-Api-Version          : {hdr['X-Api-Version']}")
    if "On-Version" in hdr:
        print(f"On-Version             : {hdr['On-Version']}")
    if "X-Request-ID" in hdr:
        print(f"X-Request-ID           : {hdr['X-Request-ID']}")
    print(
        f"X-Rate-Limit-Remaining : {hdr.get('X-Rate-Limit-Remaining', 'N/A')}"
    )
    print(f"Retry-After(seconds)   : {hdr.get('Retry-After', 'N/A')}")
    code = 0
    try:
        if hdr.get("X-Rate-Limit-Remaining") is not None and int(
                hdr["X-Rate-Limit-Remaining"]) <= 0:
            code = 1
    except Exception:
        pass
    if hdr.get("Retry-After") is not None: code = 1
    sys.exit(code)
