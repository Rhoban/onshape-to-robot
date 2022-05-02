.DEFAULT_GOAL := help
.NOTPARALLEL:

.PHONY: help
help: ## Show usage information for this Makefile.
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort \
	| awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

.PHONY: all
all:
	@rm -rf dist/*
	python3 setup.py sdist bdist_wheel

.PHONY: build
build: ## Build the Python package.
	python3 setup.py build

.PHONY: install
install: build ## Install the Python package with the --user flag.
	python3 setup.py install --user

.PHONY: upload
upload: ## PRODUCTION: Upload the package to the main PyPI repository.
	python3 -m twine upload --repository pypi dist/*

.PHONY: upload-test
upload-test: ## STAGING: Upload the package the the test PyPI repository.
	python3 -m twine upload --repository testpypi dist/*

.PHONY: clean
clean: ## Clean all build artifacts.
	python setup.py clean --all
	rm -rf build dist onshape_to_robot.egg-info docs/build
