"""Clear the onshape-to-robot cache."""
import os

from . import onshape_api as api

# Directory used for cache.
CACHE_DIR = os.path.join(os.path.dirname(api.__file__), "cache")


def clear_cache():
    """Clear the onshape-to-robot cache."""
    print("Cleaning cache directory ({})".format(CACHE_DIR))
    os.system("rm -rf {}/*".format(CACHE_DIR))


if __name__ == "__main__":
    clear_cache()
