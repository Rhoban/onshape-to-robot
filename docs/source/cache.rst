Cache
=====

Presentation
------------

In order to re-issue the same requests to the Onshape API, ``onshape-to-robot`` caches the result of most of the requests.

.. note::

    All requests involving a **workspace** can't be cached, since they rely on live version that are subject to change.

Clearing the cache
------------------

You can clear the cache using the following command:

.. code-block:: bash

    onshape-to-robot-clear-cache