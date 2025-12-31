# R-Tree Implementation in C++

This project implements a balanced **R-Tree** for spatial indexing, featuring **Guttman's Quadratic Split** algorithm to minimize node overlap. It is designed for high-velocity spatial data insertions and optimized multi-dimensional query performance.

## Features

- **Balanced R-Tree**: Efficient spatial indexing structure.
- **Guttman's Quadratic Split**: Sophisticated split strategy to minimize MBR overlap and expansion.
- **Dynamic Scalability**: Supports dynamic insertions with $O(\log_M N)$ search complexity.
- **Advanced Queries**:
  - **Range Search**: Find all items within a query rectangle.
  - **k-Nearest Neighbors (k-NN)**: Efficiently find the $k$ closest items to a query point.
- **Templated Design**: Flexible support for arbitrary dimensions and data types.

## Project Structure

- `src/Point.h`: N-dimensional point class.
- `src/Rectangle.h`: Minimum Bounding Rectangle (MBR) implementation.
- `src/RTreeNode.h`: R-Tree node structure (leaf and internal).
- `src/RTree.h`: Core R-Tree implementation including Insert, Split, Search, and k-NN algorithms.
- `main.cpp`: Demo application verifying the implementation.

## Build and Run

This project uses CMake.

### Prerequisites
- C++17 compatible compiler
- CMake >= 3.10

### Steps

1.  Create a build directory:
    ```bash
    mkdir build
    cd build
    ```

2.  Generate build files:
    ```bash
    cmake ..
    ```

3.  Compile:
    ```bash
    make
    ```

4.  Run the demo:
    ```bash
    ./RTreeDemo
    ```

## Performance

- **Insertion**: Optimized using Quadratic Split to maintain tree quality.
- **Search**: Pruning based on MBR intersection (Range Search) and min-distance (k-NN).

## License

This project is for educational and demonstration purposes.
