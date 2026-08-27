all:
	@rm -rf dist/*
	python3 -m build

upload:
	python3 -m twine upload --repository pypi dist/*

upload-test:
	python3 -m twine upload --repository testpypi dist/*

clean:
	rm -rf build dist *.egg-info