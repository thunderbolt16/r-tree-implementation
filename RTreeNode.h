#ifndef RTREENODE_H
#define RTREENODE_H

#include "Rectangle.h"
#include <memory>
#include <vector>

template <size_t DIM, typename T> struct RTreeNode {
  bool isLeaf;
  std::vector<Rectangle<DIM>> mbrs;
  std::vector<std::shared_ptr<RTreeNode<DIM, T>>>
      children;        // For internal nodes
  std::vector<T> data; // For leaf nodes
  std::weak_ptr<RTreeNode<DIM, T>> parent;

  RTreeNode(bool leaf) : isLeaf(leaf) {}

  // Calculate the MBR of this node based on its children/data
  Rectangle<DIM> computeMBR() const {
    if (mbrs.empty())
      return Rectangle<DIM>();
    Rectangle<DIM> nodeMBR = mbrs[0];
    for (size_t i = 1; i < mbrs.size(); ++i) {
      nodeMBR = nodeMBR.enlarge(mbrs[i]);
    }
    return nodeMBR;
  }

  bool isOverflow(size_t maxEntries) const { return mbrs.size() > maxEntries; }
};

#endif // RTREENODE_H
