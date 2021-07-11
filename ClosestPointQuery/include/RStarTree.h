#pragma once
#define GLM_FORCE_INLINE 
#define GLM_FORCE_INTRINSICS
#include <vector>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

typedef glm::vec3 Point;
typedef glm::vec3 Vec3;

namespace geoutils {

	// A 3D bounding box definition with standard geometric operations.
	struct BoundingBox {
	public:
		Point min, max;
	public:
		BoundingBox() :min{ Point(FLT_MAX) }, max{ Point(-FLT_MAX) } {}
		BoundingBox(const Point& min, const Point& max) : min{ min }, max{ max } {}
		BoundingBox(const BoundingBox& other) : min{ other.min }, max{ other.max }{}
		BoundingBox& operator=(const BoundingBox& other) {
			if (this == &other) return *this;
			min = other.min;
			max = other.max;
			return *this;
		}
		bool operator==(const BoundingBox& other) const { return min == other.min && max == other.max; }
		~BoundingBox() = default;
		void reset() { min = Point(FLT_MAX); max = Point(-FLT_MAX); }
		void enlarge(const BoundingBox& other) { min = glm::min(min, other.min); max = glm::max(max, other.max); }
		const BoundingBox enlarged(const BoundingBox& other) const { return BoundingBox{ glm::min(min, other.min), glm::max(max, other.max) }; }
		bool is_overlapping(const BoundingBox& other) const { return (min.x < other.max.x&& max.x > other.min.x) && (min.y < other.max.y&& max.y > other.min.y) && (min.z < other.max.z&& max.z > other.min.z); }
		bool is_inside(const BoundingBox& other) const { return glm::min(min, other.min) == other.min && glm::max(max, other.max) == other.max; }
		bool is_enclosing(const BoundingBox& other) const { return glm::min(min, other.min) == min && glm::max(max, other.max) == max; }
		float area() const { const Vec3 edges = max - min; return edges.x * edges.y * edges.z; }
		float margin() const { const Vec3 edges = max - min; return edges.x + edges.y + edges.z; }
		float overlap(const BoundingBox& other) const {
			if (!is_overlapping(other)) return 0.f;
			const BoundingBox overlapped_region = { glm::max(min, other.min), glm::min(max, other.max) };
			return overlapped_region.area();
		}
		float distance2_from_center(const BoundingBox& other) const {
			const Point center = (min + max) / 2.f;
			const Point other_center = (other.min + other.max) / 2.f;
			return glm::distance2(center, other_center);
		}
	};

	// A base class that defines any nodes with a bounding box.
	struct Node {
	public:
		BoundingBox bound{};
	public:
		Node() = delete;
		Node(const Node&) = delete;
		Node& operator=(const Node&) = delete;
		Node(const BoundingBox& bound) : bound{ bound } {}
		virtual ~Node() = default;
	};

	// Defines a leaf node with a user-defined data entry, inherit from Node class.
	template<typename DATATYPE>
	struct LeafNode : public Node {
	public:
		DATATYPE data{};
	public:
		LeafNode() = delete;
		LeafNode(const LeafNode&) = delete;
		LeafNode& operator=(const LeafNode&) = delete;
		LeafNode(const BoundingBox& bound, DATATYPE data) : Node{ bound }, data{ data } {}
		virtual ~LeafNode() override {}
		void* operator new(size_t i) { return _mm_malloc(i, 16); }
		void operator delete(void* p) { _mm_free(p); }
	};

	// Defines an internal node that stores multiple nodes, which can only be one of the types either InternalNode or LeafNode, inherit from Node class.
	template<typename DATATYPE, int MAX_NODE, int MIN_NODE>
	struct InternalNode : public Node {
	public:
		bool has_leaves = false; // Indicate whether its children are in type LeafNode.
		std::vector<Node*> children{};
	public:
		InternalNode(const BoundingBox& bound) : Node{ bound } { children.reserve(MIN_NODE); }
		InternalNode(const InternalNode&) = delete;
		InternalNode& operator=(const InternalNode&) = delete;
		void* operator new(size_t i) { return _mm_malloc(i, 16); }
		void operator delete(void* p) { _mm_free(p); }
		virtual ~InternalNode() override {
			for (size_t i = 0; i < children.size(); ++i) {
				if (children[i] != nullptr) {
					delete children[i];
					children[i] = nullptr;
				}
			}
		}
	};

	// A 3D R*-Tree acceleration structure for spatial storage and query. 
	// An implementation by following the paper https://epub.ub.uni-muenchen.de/4256/1/31.pdf by N Beckmann et al.
	template<typename DATATYPE, int MAX_NODE = 64, int MIN_NODE = static_cast<int>(0.4 * static_cast<double>(MAX_NODE))>
	class RStarTree {
		using LeafNode = LeafNode<DATATYPE>;
		using InternalNode = InternalNode<DATATYPE, MAX_NODE, MIN_NODE>;
		const size_t CHOOSE_SUBTREE_P = MAX_NODE / 2;
		const size_t REINSERT_P = static_cast<size_t>(0.3 * static_cast<double>(MAX_NODE));
		static_assert(MIN_NODE > 0 && MAX_NODE > 0 && MIN_NODE <= MAX_NODE, "Invalid MIN_NODE or MAX_NODE value for RStarTree");
	private:
		InternalNode* root = nullptr;
		size_t size = 0;
	public:
		RStarTree() = default;
		RStarTree(const RStarTree&) = delete;
		RStarTree& operator=(const RStarTree&) = delete;
		~RStarTree() {
			// Free up memory recursively using virtual destructors.
			if (root != nullptr) {
				delete root;
				root = nullptr;
			}
		}
		// Get the number of leaf nodes of the constructed tree.
		const size_t count() const { return size; }
		// Retrive the bounding box of the constructed tree.
		const BoundingBox bound() const { return root->bound; }
		// Insert an entry to the structure with a specified bounding box.
		void insert(const Point& min, const Point& max, DATATYPE data) {
			const BoundingBox bound{ min, max };
			LeafNode* new_leaf = new LeafNode(bound, data);
			if (root == nullptr) {
				root = new InternalNode(bound);
				root->has_leaves = true;
				root->children.push_back(new_leaf);
			}
			else {
				insert_internal(new_leaf, root, true);
			}
			size++;
		}
		// Breadth-first traversal, useful for printing out tree visualization.
		// Template Argument:
		//	callback: Callback function that accept (int, Node*) parameters. See example for usage.
		// Example:
		//	const auto callback = [&](int layer, Node* node) { /* process node info. */ };
		//	tree.traverse_bfs(callback);
		template<typename Func>
		void traverse_bfs(Func callback) const { traverse_bfs_internal(callback, root, 0); }
		// Depth-first traversal, invoked on every entries that intersect within the searching radius.
		// Template Argument:
		//	callback: A callable functor that accept (DATATYPE) parameter. See example for usage.
		// Example:
		//	RStarTree<Triangle*, 64> tree;
		//	/* insert Triangle* entries into tree */
		//	const auto callback = [&](Triangle* tri) { /* process triangle info. */ };
		//	tree.search_radius(Point{0.f, 0.f, 0.f}, 1.f, callback);
		template<typename Func>
		void search_radius(const Point& query_point, float max_dist, Func callback) const { search_radius_internal(query_point, max_dist, callback, root); }
	private:
		// A recursive function for inserting a leaf node to the optimal subtrees.
		InternalNode* insert_internal(LeafNode* leaf, InternalNode* node, bool first_insert) {
			// Include the leaf node into the node's bounding box.
			node->bound.enlarge(leaf->bound);
			if (node->has_leaves) {
				// Directly insert into node if it's in the leaf level.
				node->children.push_back(leaf);
			}
			else {
				// Find the best subtree to insert into, recursively insert the leaf node to the bottom of the tree.
				InternalNode* best_subtree = choose_subtree(node, leaf->bound);
				InternalNode* overflow_node = insert_internal(leaf, best_subtree, first_insert);
				// Handle any overflow node coming from the child level.
				if (overflow_node == nullptr) return nullptr;
				node->children.push_back(overflow_node);
			}
			// Overflow treatment.
			if (node->children.size() > MAX_NODE) {
				assert(node->children.size() == MAX_NODE + 1 && "Overflow children must have a size of MAX_NODE + 1.");
				// Opportunistic reinsertion in hope of constructing a better performing tree.
				if (node != root && first_insert) {
					reinsert(node);
					return nullptr;
				}
				// Split the node, create a new root and reparent if it's the root node.
				InternalNode* split_node = split(node);
				if (node == root) {
					InternalNode* new_root = new InternalNode(BoundingBox{});
					new_root->children.push_back(root);
					new_root->children.push_back(split_node);
					std::for_each(new_root->children.begin(), new_root->children.end(), EnlargeBoundingBox(new_root->bound));
					root = new_root;
					return nullptr;
				}
				// Propagate the splitted node upwards.
				return split_node;
			}
			return nullptr;
		}
		// A recursive function for visiting all nodes in a breadth-first manner.
		template<typename Func>
		void traverse_bfs_internal(Func f, InternalNode* node, int layer) const {
			if (node == nullptr) return;
			for (size_t i = 0; i < node->children.size(); ++i) {
				f(layer, node->children[i]);
			}
			for (size_t i = 0; i < node->children.size(); ++i) {
				traverse_bfs_internal(f, dynamic_cast<InternalNode*>(node->children[i]), layer + 1);
			}
		}
		// A recursive function for searching leaf nodes that overlap with the proximity defined by query_point and max_dist.
		template<typename Func>
		bool search_radius_internal(const Point& query_point, float max_dist, Func callback, InternalNode* node) const {
			assert(node != nullptr);
			for (size_t i = 0; i < node->children.size(); ++i) {
				// Sphere-AABB intersection check, terminate early if there's no overlap.
				const Vec3 a = glm::max(glm::min(query_point, node->children[i]->bound.max), node->children[i]->bound.min);
				const float distance = glm::length(a - query_point);
				if (distance > max_dist) continue;
				LeafNode* leaf = dynamic_cast<LeafNode*>(node->children[i]);
				if (leaf) {
					callback(leaf->data);
				}
				else {
					search_radius_internal(query_point, max_dist, callback, static_cast<InternalNode*>(node->children[i]));
				}
			}
			return true;
		}
		// Choosing the optimal child node that has the minimal impact (overlapping).
		InternalNode* choose_subtree(InternalNode* node, const BoundingBox& bound) {
			assert(node != nullptr);
			assert(!node->has_leaves && "Leaf nodes are already handled in insert_internal(), subtree must contain no leaves.");

			// Child nodes point to leaves. Choose the minimum overlap enlargement subtree.
			if (static_cast<InternalNode*>(node->children[0])->has_leaves) {
				// Determine the minimum overlap cost.
				if (MAX_NODE > (CHOOSE_SUBTREE_P * 2) / 3 && node->children.size() > CHOOSE_SUBTREE_P) {
					std::partial_sort(node->children.begin(), node->children.begin() + CHOOSE_SUBTREE_P, node->children.end(), SortByArea(bound));
					std::vector<Node*> nodes(node->children.begin(), node->children.begin() + CHOOSE_SUBTREE_P);
					return static_cast<InternalNode*>(min_overlap_enlargement_node(nodes, bound));
				}
				return static_cast<InternalNode*>(min_overlap_enlargement_node(node->children, bound));
			}
			// Child nodes are internal nodes. Choose the minimum area subtree.
			return static_cast<InternalNode*>(min_area_enlargement_node(node->children, bound));
		}
		// Splitting the specified node into two halves. Return the newly splitted node, leaving the input node as the other half.
		InternalNode* split(InternalNode* node) {
			const size_t distribution_count = MAX_NODE - 2 * MIN_NODE + 2;
			assert(node != nullptr);
			assert(node->children.size() == MAX_NODE + 1 && "Node size should be overflowed by one.");
			assert(distribution_count > 0 && "Distribution count must be positive.");
			assert(MIN_NODE + distribution_count - 1 <= node->children.size() && "Invalid distribution count.");

			// For each axis, sort the entries by lower then by the upper bound value.
			// Then determine the best splitting axis by the child nodes minimum margin sum value.
			const uint8_t INVALID_AXIS = 3;
			uint8_t best_split_axis = INVALID_AXIS;
			float least_margin = FLT_MAX;
			for (uint8_t axis = 0; axis < 3; ++axis) {
				std::sort(node->children.begin(), node->children.end(), SortByBoundMin(axis));
				std::sort(node->children.begin(), node->children.end(), SortByBoundMax(axis));
				float margin = 0.f;
				for (size_t k = 0; k < distribution_count; ++k) {
					BoundingBox left{};
					BoundingBox right{};
					std::for_each(node->children.begin(), node->children.begin() + (MIN_NODE + k), EnlargeBoundingBox(left));
					std::for_each(node->children.begin() + (MIN_NODE + k + 1), node->children.end(), EnlargeBoundingBox(right));
					margin += left.margin() + right.margin();
				}
				if (margin < least_margin) {
					best_split_axis = axis;
					least_margin = margin;
				}
			}
			assert(least_margin != FLT_MAX && best_split_axis != INVALID_AXIS && "Invalid split axis.");

			// For each distribution along the best axis, determine the best splitting index by the child nodes minimum overlap value.
			// Resolve ties by their minimum area sum value.
			const size_t INVALID_DISTRIBUTION = -1;
			float least_overlap = FLT_MAX;
			float least_area = FLT_MAX;
			size_t best_distribution = INVALID_DISTRIBUTION;
			for (size_t k = 0; k < distribution_count; ++k) {
				BoundingBox left{};
				BoundingBox right{};
				std::for_each(node->children.begin(), node->children.begin() + (MIN_NODE + k), EnlargeBoundingBox(left));
				std::for_each(node->children.begin() + (MIN_NODE + k + 1), node->children.end(), EnlargeBoundingBox(right));
				const float overlap = left.overlap(right);
				const float area = left.area() + right.area();
				if (area < least_area) {
					least_area = area;
				}
				if (overlap < least_overlap || (overlap == least_overlap && area == least_area)) {
					least_overlap = overlap;
					best_distribution = k;
				}
			}
			assert(least_overlap != FLT_MAX && least_area != FLT_MAX && best_distribution != INVALID_DISTRIBUTION && "Invalid split distribution.");

			// Recreate the optimal split with the results found above and perform the split.
			std::sort(node->children.begin(), node->children.end(), SortByBoundMin(best_split_axis));
			std::sort(node->children.begin(), node->children.end(), SortByBoundMax(best_split_axis));
			InternalNode* new_node = new InternalNode(BoundingBox{});
			new_node->has_leaves = node->has_leaves;
			new_node->children.assign(node->children.begin() + (MIN_NODE + best_distribution + 1), node->children.end());
			node->children.erase(node->children.begin() + (MIN_NODE + best_distribution + 1), node->children.end());

			// Update the bounding box for the 'left' node
			node->bound.reset();
			std::for_each(node->children.begin(), node->children.end(), EnlargeBoundingBox(node->bound));

			// Update the bounding box for the 'right' node
			new_node->bound.reset();
			std::for_each(new_node->children.begin(), new_node->children.end(), EnlargeBoundingBox(new_node->bound));

			return new_node; // Return the 'right' node, leaving the input node as the 'left' node.
		}
		// An opportunistic reinsertion in hope of constructing a better performing tree by reinserting leaf nodes.
		// Since depending on the order of insertion during construction, prior grouping and splitting results might not be in an optimal distribution. 
		void reinsert(InternalNode* node) {
			assert(node != nullptr);
			assert(node->has_leaves && "Children must be LeafNodes.");
			assert(node->children.size() == MAX_NODE + 1 && "Only nodes with MAX_NODE + 1 size is qualified to perform a reinsertion");
			const size_t p = std::min(std::max(REINSERT_P, (size_t)1), (size_t)MAX_NODE);

			// Sort the child nodes by the distance from the node center, pruning the furthest children.
			std::sort(node->children.begin(), node->children.end(), SortByDistanceFromCenter(node->bound));
			std::vector<Node*> pruned_nodes(node->children.end() - p, node->children.end());
			node->children.erase(node->children.end() - p, node->children.end());

			// Update the node bounding box.
			node->bound.reset();
			std::for_each(node->children.begin(), node->children.end(), EnlargeBoundingBox(node->bound));

			// Reinsert the marked nodes at the root level.
			for (size_t i = 0; i < pruned_nodes.size(); ++i) {
				assert(dynamic_cast<LeafNode*>(pruned_nodes[i]) != nullptr && "Only leaf nodes can be reinserted.");
				insert_internal(static_cast<LeafNode*>(pruned_nodes[i]), root, false);
			}
		}
		// Find the minimum area enlargement node given a collection of nodes and a bound to be inserted into.
		Node* min_area_enlargement_node(const std::vector<Node*>& nodes, const BoundingBox& bound) const {
			Node* best_node = nullptr;
			float least_area = FLT_MAX;
			for (size_t i = 0; i < nodes.size(); ++i) {
				const BoundingBox enlarged_bound = nodes[i]->bound.enlarged(bound);
				const float enlarged_area = enlarged_bound.area() - nodes[i]->bound.area();
				if (enlarged_area < least_area) {
					least_area = enlarged_area;
					best_node = nodes[i];
				}
			}
			assert(best_node != nullptr && least_area != FLT_MAX && "Invalid bounds or empty collection of nodes.");
			return best_node;
		}
		// Find the minimum overlapping enlargement node given a collection of nodes and a bound to be inserted into.
		Node* min_overlap_enlargement_node(const std::vector<Node*>& nodes, const BoundingBox& bound) const {
			Node* best_node = nullptr;
			float least_overlap = FLT_MAX;
			// For each node, expand the node to include the input bound. And for each expanded bound, sum up the overlapped volume with other sibling nodes.
			// Pick the minimum overlapping enlargement node as the return value.
			// Note: This is an expensive O(n2) operation, optimize this algorithm if possible.
			for (size_t i = 0; i < nodes.size(); ++i) {
				float overlap = 0.f;
				const BoundingBox enlarged_bound = nodes[i]->bound.enlarged(bound);
				for (size_t j = 0; j < nodes.size(); ++j) {
					if (i == j) continue;
					overlap += enlarged_bound.overlap(nodes[j]->bound) - nodes[i]->bound.overlap(nodes[j]->bound);
				}
				if (overlap < least_overlap) {
					least_overlap = overlap;
					best_node = nodes[i];
				}
			}
			assert(best_node != nullptr && least_overlap != FLT_MAX && "Invalid bounds or empty collection of nodes.");
			return best_node;
		}
	private:
		// Capturing lambda expressions declaration
		struct SortByBoundMin {
			const uint8_t axis;
			explicit SortByBoundMin(uint8_t axis) : axis{ axis } {}
			bool operator()(const Node* a, const Node* b) const { return a->bound.min[axis] < b->bound.min[axis]; }
		};
		struct SortByBoundMax {
			const uint8_t axis;
			explicit SortByBoundMax(uint8_t axis) : axis{ axis } {}
			bool operator()(const Node* a, const Node* b) const { return a->bound.max[axis] < b->bound.max[axis]; }
		};
		struct SortByArea {
			const float area;
			explicit SortByArea(const BoundingBox& bound) : area{ bound.area() } {}
			bool operator() (const Node* a, const Node* b) const { return area - a->bound.area() < area - b->bound.area(); }
		};
		struct SortByDistanceFromCenter {
			const BoundingBox& bound;
			explicit SortByDistanceFromCenter(const BoundingBox& bound) : bound{ bound } {}
			bool operator() (const Node* a, const Node* b) const { return a->bound.distance2_from_center(bound) < b->bound.distance2_from_center(bound); }
		};
		struct EnlargeBoundingBox {
			BoundingBox& bound;
			explicit EnlargeBoundingBox(BoundingBox& bound) : bound{ bound } {}
			void operator()(const Node* node) { bound.enlarge(node->bound); }
		};
	};

} //namespace geoutils