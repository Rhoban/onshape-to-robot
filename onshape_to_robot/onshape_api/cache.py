import os
import inspect
import hashlib
from pathlib import Path
import pickle


def get_cache_path() -> Path:
    """
    Return the path to the user cache.
    """
    path = Path.home() / ".cache" / "onshape-to-robot"
    path.mkdir(parents=True, exist_ok=True)
    return path


def can_cache(method, *args, **kwargs) -> bool:
    """
    Check if the cache can be used.
    When using wmv=w, the current workspace is used, which make it impossible to cache.
    """
    signature = inspect.signature(method)
    wmv = None
    if "wmv" in signature.parameters:
        wmv = signature.parameters["wmv"].default
    if "wmv" in kwargs:
        wmv = kwargs["wmv"]
    return wmv != "w"


def cache_response(method):
    """
    Decorator that caches the response of a method.
    """

    def cached_call(*args, **kwargs):
        # Checking if the method can be cached
        if not can_cache(method, *args, **kwargs):
            return method(*args, **kwargs)

        # Building filename that is unique for method/args combination
        method_name = method.__qualname__
        arguments = {"args": args, "kwargs": kwargs}
        arguments_hash = hashlib.sha1(pickle.dumps(arguments)).hexdigest()
        filename = f"{get_cache_path()}/{method_name}_{arguments_hash}.pkl"

        if not os.path.exists(filename):
            result = method(*args, **kwargs)
            with open(filename, "wb") as f:
                pickle.dump(result, f)

        return pickle.load(open(filename, "rb"))

    return cached_call
