"""Clear the onshape-to-robot cache."""
import shutil

from .onshape_api.client import Client


def main():
    """Clear the onshape-to-robot cache."""
    cache_dir = Client.get_cache_path()
    print("Removing cache directory: {}".format(cache_dir))
    shutil.rmtree(cache_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
