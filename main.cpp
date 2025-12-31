#include "src/RTree.h"
#include <iostream>
#include <random>
#include <vector>

struct Data {
  int id;
  Point<2> location;

  bool operator==(const Data &other) const { return id == other.id; }
};

std::ostream &operator<<(std::ostream &os, const Data &d) {
  os << "ID: " << d.id << " (" << d.location.get(0) << ", " << d.location.get(1)
     << ")";
  return os;
}

int main() {
  // R-Tree with 2D points, storing Data objects, min entries 2, max entries 4
  RTree<2, Data, 2, 4> tree;

  std::vector<Data> allData;

  // Insert some data
  std::cout << "Inserting 20 random points..." << std::endl;
  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis(0.0, 100.0);

  for (int i = 0; i < 20; ++i) {
    Point<2> p = {dis(gen), dis(gen)};
    Data d = {i, p};
    tree.insert(p, d);
    allData.push_back(d);
    std::cout << "Inserted: " << d << std::endl;
  }

  // Range Search
  std::cout << "\n--- Range Search ---" << std::endl;
  Rectangle<2> searchRect({20.0, 20.0}, {50.0, 50.0});
  std::cout << "Searching in rect: (" << searchRect.minPoint.get(0) << ","
            << searchRect.minPoint.get(1) << ") to ("
            << searchRect.maxPoint.get(0) << "," << searchRect.maxPoint.get(1)
            << ")" << std::endl;

  std::vector<Data> rangeResults = tree.search(searchRect);
  std::cout << "Found " << rangeResults.size() << " items:" << std::endl;
  for (const auto &res : rangeResults) {
    std::cout << res << std::endl;
    // Verify
    if (!searchRect.contains(res.location)) {
      std::cout << "ERROR: Result not in range!" << std::endl;
    }
  }

  // k-NN Search
  std::cout << "\n--- k-NN Search ---" << std::endl;
  Point<2> queryPoint = {50.0, 50.0};
  int k = 5;
  std::cout << "Finding " << k << " nearest neighbors to (" << queryPoint.get(0)
            << ", " << queryPoint.get(1) << ")" << std::endl;

  std::vector<Data> knnResults = tree.nearestNeighbor(queryPoint, k);
  std::cout << "Found " << knnResults.size() << " items:" << std::endl;
  for (const auto &res : knnResults) {
    std::cout << res << " Dist: " << res.location.distance(queryPoint)
              << std::endl;
  }

  // Brute force check for k-NN
  std::sort(allData.begin(), allData.end(), [&](const Data &a, const Data &b) {
    return a.location.distanceSquared(queryPoint) <
           b.location.distanceSquared(queryPoint);
  });

  std::cout << "\nBrute force top " << k << ":" << std::endl;
  for (int i = 0; i < k && i < allData.size(); ++i) {
    std::cout << allData[i]
              << " Dist: " << allData[i].location.distance(queryPoint)
              << std::endl;
  }

  return 0;
}
