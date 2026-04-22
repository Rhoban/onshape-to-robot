"""Clear the onshape-to-robot cache."""


def main():
    import shutil

    from .onshape_api.cache import get_cache_path
    cache_dir = get_cache_path()
    print(f"Removing cache directory: {cache_dir}")
    shutil.rmtree(cache_dir, ignore_errors=True)

if __name__ == "__main__":
    import argparse

    import argcomplete
    parser = argparse.ArgumentParser(
        description="Clear the onshape-to-robot cache directory.",
        usage="onshape-to-robot-clear-cache"
    )
    argcomplete.autocomplete(parser)
    parser.parse_args()
    main()
