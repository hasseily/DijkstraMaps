# DijkstraMaps
Test code to generated weigthed Dijkstra Maps.

This code generates a 32x32 Dijkstra Map based off of a 32x32 map of extra movement costs and walls.
It's quick, it's dirty, and it's guaranteed not to be production ready.

It also runs 1,000 iterations of the code for benchmarking before running it once and outputting the result on the console.

If you'd like to try the GPU version, there's another repo with a GPU-based version. In general the GPU won't be worth it below at least 64x64 maps.
