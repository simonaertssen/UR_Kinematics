# Run me with make

# For cleaning up after yourself:
# all: make clean

all: make

make:
	python setup.py build_ext --inplace
	python main.py

clean:
	@rm *.so
	@rm *.html
	@rm *.c
	@rm -rf __pycache__
	@rm -rf build
	@echo "Clean done."