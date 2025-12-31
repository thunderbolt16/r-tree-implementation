#ifndef RTREE_H
#define RTREE_H

#include "RTreeNode.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

template <size_t DIM, typename T, size_t MIN_ENTRIES, size_t MAX_ENTRIES>
class RTree {
public:
  using Node = RTreeNode<DIM, T>;
  using NodePtr = std::shared_ptr<Node>;

  NodePtr root;

  RTree() { root = std::make_shared<Node>(true); }

  void insert(const Point<DIM> &point, const T &value) {
    Rectangle<DIM> rect(point);
    NodePtr leaf = chooseLeaf(root, rect);
    leaf->mbrs.push_back(rect);
    leaf->data.push_back(value);

    if (leaf->isOverflow(MAX_ENTRIES)) {
      splitNode(leaf);
    } else {
      adjustTree(leaf);
    }
  }

  std::vector<T> search(const Rectangle<DIM> &queryRect) {
    std::vector<T> results;
    searchRecursive(root, queryRect, results);
    return results;
  }

  // k-NN Search
  struct NNEntry {
    double dist;
    NodePtr node;
    size_t index; // Index in node's entries (if leaf) or children (if internal)
    bool isData;  // True if this entry represents a data point in a leaf

    bool operator>(const NNEntry &other) const { return dist > other.dist; }
  };

  std::vector<T> nearestNeighbor(const Point<DIM> &queryPoint, size_t k) {
    std::vector<T> results;
    std::priority_queue<NNEntry, std::vector<NNEntry>, std::greater<NNEntry>>
        pq;

    // Add root children to PQ
    // We treat the root itself as a starting point.
    // Actually, standard approach: put root in PQ with dist 0?
    // Or better: expand root immediately.

    // Let's put the root node itself as a generic entry
    // We need a slight modification to NNEntry to handle "Node" vs "Data Item"
    // But our nodes have multiple entries.
    // Let's just push the root node.

    // Simplified approach:
    // PQ stores (distance, NodePtr). If NodePtr is leaf, we iterate its points.
    // Wait, we need to explore children in order of distance.
    // So PQ should store (distance, NodePtr) OR (distance, DataItem).

    // Let's refine NNEntry:
    // It can represent a Node (to be explored) or a Data Point (to be
    // returned).

    // For the root, distance is 0 (it covers everything conceptually, or use
    // MBR distance). Since root covers all, dist is 0.

    // Actually, we should push all entries of the root into the PQ.
    // But root might be a leaf.

    // Let's use a helper.
    // We need to handle the fact that we want to return T, but we need to track
    // Nodes.

    // Re-designing NNEntry for the queue:
    // It holds a pointer to a node (if internal) or is a terminal data point.
    // But we don't have a pointer to a single data point easily without index.

    // Let's use:
    // NodePtr node; (if null, then it's a data point?)
    // int index; (index in the parent node, or index of data)
    // This is getting complicated.

    // Alternative:
    // PQ stores `(mindist, NodePtr)`.
    // When we pop a NodePtr:
    //   If internal: calculate dist for all children MBRs, push to PQ.
    //   If leaf: calculate dist for all data points, push to PQ (as a special
    //   "DataNode" or similar wrapper?).

    // Let's try:
    // PQ stores `(dist, NodePtr, index_in_node)`.
    // If `NodePtr` is null, it means `index_in_node` is just a placeholder? No.

    // Let's stick to:
    // PQ stores `(dist, NodePtr)`.
    // When popping an internal node, push all children.
    // When popping a leaf node, push all data points?
    // Data points need to be distinguished.

    // Let's use a wrapper struct that can hold either a NodePtr or a T (with
    // Point). Since T is generic, maybe just store NodePtr and index. If index
    // == -1, it refers to the Node itself (to be expanded). If index >= 0, it
    // refers to data[index] in the Node (which must be a leaf).

    struct QueueItem {
      double dist;
      NodePtr node;
      int index; // -1 if it's the node itself, >=0 if it's a data item in the
                 // node

      bool operator>(const QueueItem &other) const { return dist > other.dist; }
    };

    std::priority_queue<QueueItem, std::vector<QueueItem>,
                        std::greater<QueueItem>>
        queue;

    // Start with root
    // We can't calculate distance to root MBR easily if we don't store it in
    // the node itself (we compute it). But root MBR covers everything usually.
    // Let's just push root with dist 0.
    queue.push({0.0, root, -1});

    while (!queue.empty() && results.size() < k) {
      QueueItem item = queue.top();
      queue.pop();

      if (item.index != -1) {
        // It's a data point
        results.push_back(item.node->data[item.index]);
      } else {
        // It's a node to expand
        NodePtr n = item.node;
        if (n->isLeaf) {
          for (size_t i = 0; i < n->mbrs.size(); ++i) {
            double d = n->mbrs[i].distanceSquared(queryPoint);
            queue.push({d, n, (int)i});
          }
        } else {
          for (size_t i = 0; i < n->mbrs.size(); ++i) {
            double d = n->mbrs[i].distanceSquared(queryPoint);
            queue.push({d, n->children[i], -1});
          }
        }
      }
    }

    return results;
  }

private:
  void searchRecursive(NodePtr node, const Rectangle<DIM> &queryRect,
                       std::vector<T> &results) {
    if (node->isLeaf) {
      for (size_t i = 0; i < node->mbrs.size(); ++i) {
        if (queryRect.intersects(node->mbrs[i])) {
          results.push_back(node->data[i]);
        }
      }
    } else {
      for (size_t i = 0; i < node->mbrs.size(); ++i) {
        if (queryRect.intersects(node->mbrs[i])) {
          searchRecursive(node->children[i], queryRect, results);
        }
      }
    }
  }

  NodePtr chooseLeaf(NodePtr node, const Rectangle<DIM> &rect) {
    if (node->isLeaf)
      return node;

    double minExpansion = std::numeric_limits<double>::max();
    double minArea = std::numeric_limits<double>::max();
    int bestIndex = -1;

    for (size_t i = 0; i < node->mbrs.size(); ++i) {
      double expansion = node->mbrs[i].expansionNeeded(rect);
      double area = node->mbrs[i].area();

      if (expansion < minExpansion) {
        minExpansion = expansion;
        minArea = area;
        bestIndex = i;
      } else if (expansion == minExpansion) {
        if (area < minArea) {
          minArea = area;
          bestIndex = i;
        }
      }
    }

    return chooseLeaf(node->children[bestIndex], rect);
  }

  void adjustTree(NodePtr node) {
    if (node == root)
      return; // Root has no parent to adjust

    NodePtr p = node->parent.lock();
    if (!p)
      return;

    // Find entry for 'node' in 'p' and update its MBR
    for (size_t i = 0; i < p->children.size(); ++i) {
      if (p->children[i] == node) {
        p->mbrs[i] = node->computeMBR();
        break;
      }
    }

    adjustTree(p);
  }

  // Guttman's Quadratic Split
  void splitNode(NodePtr node) {
    // Create a new node
    NodePtr newNode = std::make_shared<Node>(node->isLeaf);
    newNode->parent = node->parent;

    // All entries to distribute
    std::vector<Rectangle<DIM>> allMBRs = node->mbrs;
    // We need to store data/children too
    std::vector<T> allData;
    std::vector<NodePtr> allChildren;

    if (node->isLeaf) {
      allData = node->data;
    } else {
      allChildren = node->children;
    }

    // Clear current node
    node->mbrs.clear();
    node->data.clear();
    node->children.clear();

    // Pick Seeds
    size_t seed1, seed2;
    pickSeeds(allMBRs, seed1, seed2);

    // Assign seeds
    // Group 1: node, Group 2: newNode
    addToNode(node, allMBRs[seed1], allData, allChildren, seed1);
    addToNode(newNode, allMBRs[seed2], allData, allChildren, seed2);

    std::vector<bool> assigned(allMBRs.size(), false);
    assigned[seed1] = true;
    assigned[seed2] = true;
    size_t assignedCount = 2;

    // Distribute remaining entries
    while (assignedCount < allMBRs.size()) {
      // Check if one group needs all remaining entries to meet MIN_ENTRIES
      size_t remaining = allMBRs.size() - assignedCount;
      if (node->mbrs.size() + remaining == MIN_ENTRIES) {
        // Assign all to node
        for (size_t i = 0; i < allMBRs.size(); ++i) {
          if (!assigned[i])
            addToNode(node, allMBRs[i], allData, allChildren, i);
        }
        break;
      }
      if (newNode->mbrs.size() + remaining == MIN_ENTRIES) {
        // Assign all to newNode
        for (size_t i = 0; i < allMBRs.size(); ++i) {
          if (!assigned[i])
            addToNode(newNode, allMBRs[i], allData, allChildren, i);
        }
        break;
      }

      // Pick Next
      size_t next = pickNext(allMBRs, assigned, node->computeMBR(),
                             newNode->computeMBR());

      // Decide which group
      Rectangle<DIM> group1MBR = node->computeMBR();
      Rectangle<DIM> group2MBR = newNode->computeMBR();

      double d1 = group1MBR.expansionNeeded(allMBRs[next]);
      double d2 = group2MBR.expansionNeeded(allMBRs[next]);

      if (d1 < d2) {
        addToNode(node, allMBRs[next], allData, allChildren, next);
      } else if (d2 < d1) {
        addToNode(newNode, allMBRs[next], allData, allChildren, next);
      } else {
        // Tie-breaker: area, then count
        if (group1MBR.area() < group2MBR.area()) {
          addToNode(node, allMBRs[next], allData, allChildren, next);
        } else if (group2MBR.area() < group1MBR.area()) {
          addToNode(newNode, allMBRs[next], allData, allChildren, next);
        } else {
          if (node->mbrs.size() < newNode->mbrs.size()) {
            addToNode(node, allMBRs[next], allData, allChildren, next);
          } else {
            addToNode(newNode, allMBRs[next], allData, allChildren, next);
          }
        }
      }
      assigned[next] = true;
      assignedCount++;
    }

    // Handle parent
    if (node == root) {
      NodePtr newRoot = std::make_shared<Node>(false);
      newRoot->children.push_back(node);
      newRoot->mbrs.push_back(node->computeMBR());
      newRoot->children.push_back(newNode);
      newRoot->mbrs.push_back(newNode->computeMBR());

      node->parent = newRoot;
      newNode->parent = newRoot;
      root = newRoot;
    } else {
      NodePtr p = node->parent.lock();
      // Update node's MBR in parent (it might have shrunk/changed)
      // Actually, we just add the newNode to parent and let parent adjust/split
      // But we need to update 'node's entry in parent first?
      // Yes, 'node' is already in parent, but its MBR changed.
      // We will fix 'node's MBR in parent in adjustTree or here?
      // Let's do it here.

      // Find node in parent
      for (size_t i = 0; i < p->children.size(); ++i) {
        if (p->children[i] == node) {
          p->mbrs[i] = node->computeMBR();
          break;
        }
      }

      // Add newNode to parent
      p->children.push_back(newNode);
      p->mbrs.push_back(newNode->computeMBR());
      newNode->parent = p;

      if (p->isOverflow(MAX_ENTRIES)) {
        splitNode(p);
      } else {
        adjustTree(p);
      }
    }
  }

  void addToNode(NodePtr n, const Rectangle<DIM> &rect,
                 const std::vector<T> &allData,
                 const std::vector<NodePtr> &allChildren, size_t index) {
    n->mbrs.push_back(rect);
    if (n->isLeaf) {
      n->data.push_back(allData[index]);
    } else {
      n->children.push_back(allChildren[index]);
      allChildren[index]->parent = n;
    }
  }

  void pickSeeds(const std::vector<Rectangle<DIM>> &entries, size_t &seed1,
                 size_t &seed2) {
    double maxWaste = -1.0;

    for (size_t i = 0; i < entries.size(); ++i) {
      for (size_t j = i + 1; j < entries.size(); ++j) {
        Rectangle<DIM> combined = entries[i].enlarge(entries[j]);
        double waste = combined.area() - entries[i].area() - entries[j].area();
        if (waste > maxWaste) {
          maxWaste = waste;
          seed1 = i;
          seed2 = j;
        }
      }
    }
  }

  size_t pickNext(const std::vector<Rectangle<DIM>> &entries,
                  const std::vector<bool> &assigned,
                  const Rectangle<DIM> &group1MBR,
                  const Rectangle<DIM> &group2MBR) {
    double maxDiff = -1.0;
    size_t bestIndex = 0;

    for (size_t i = 0; i < entries.size(); ++i) {
      if (assigned[i])
        continue;

      double d1 = group1MBR.expansionNeeded(entries[i]);
      double d2 = group2MBR.expansionNeeded(entries[i]);
      double diff = std::abs(d1 - d2);

      if (diff > maxDiff) {
        maxDiff = diff;
        bestIndex = i;
      }
    }
    return bestIndex;
  }
};

#endif // RTREE_H
