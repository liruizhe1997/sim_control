rm -rf build
mkdir build
cmake -B build -S src &&
cd build &&
make &&
cd ..
