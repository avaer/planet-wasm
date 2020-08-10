mkdir -p bin
emcc --std=c++11 -s WASM=1 -O3 -s TOTAL_MEMORY=67108864 \
  util.cc FastNoise.cpp noise.cc march.cc vector.cc objectize.cc \
  --pre-js prefix.js --post-js postfix.js \
  -o bin/objectize.js
cat prefix2.js bin/objectize.js >bin/objectize2.js
