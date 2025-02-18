"""Clear the onshape-to-robot cache."""

import shutil

from .onshape_api.cache import get_cache_path


def main():
    """Clear the onshape-to-robot cache."""
    cache_dir = get_cache_path()
    print("Removing cache directory: {}".format(cache_dir))
    shutil.rmtree(cache_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
