#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
    namespace SceneObjects {

        BVHAccel::BVHAccel(const std::vector<Primitive*>& _primitives,
            size_t max_leaf_size) {

            primitives = std::vector<Primitive*>(_primitives);
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
        }

        BVHAccel::~BVHAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        void BVHAccel::draw(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->draw(c, alpha);
                }
            }
            else {
                draw(node->l, c, alpha);
                draw(node->r, c, alpha);
            }
        }

        void BVHAccel::drawOutline(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->drawOutline(c, alpha);
                }
            }
            else {
                drawOutline(node->l, c, alpha);
                drawOutline(node->r, c, alpha);
            }
        }

        BVHNode* BVHAccel::construct_bvh(std::vector<Primitive*>::iterator start,
            std::vector<Primitive*>::iterator end,
            size_t max_leaf_size) {

            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.


            BBox bbox;

            // construct the root bonding box
            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                bbox.expand(bb);
            }

            // create the root node
            BVHNode* node = new BVHNode(bbox);
            // check if no further split needed
            size_t num_primitives = distance(start, end);
            if (num_primitives <= max_leaf_size) {
                // base case
                node->start = start;
                node->end = end;
                return node;
            }

            // split into left and right sub trees
            BBox centroid_bbox;
            for (auto p = start; p != end; p++) {
                centroid_bbox.expand((*p)->get_bbox().centroid());
            }
            // decide the split axis
            double x = centroid_bbox.extent.x;
            double y = centroid_bbox.extent.y;
            double z = centroid_bbox.extent.z;
            int split_axis;
            if (x > y && x > z) {
                split_axis = 0;
            }
            else if (y > x && y > z) {
                split_axis = 1;
            }
            else {
                split_axis = 2;
            }

            // sort the iterator such that all primitives belong to the same branch are on the same side
            std::sort(start, end, [split_axis](Primitive* a, Primitive* b) {
                return a->get_bbox().centroid()[split_axis] < b->get_bbox().centroid()[split_axis];
                });

            // get the mid
            // since spliting from the middle, either branch should not be empty
            std::vector<Primitive*>::iterator mid = start + num_primitives / 2;

            // recursively construct the bvh tree
            node->l = construct_bvh(start, mid, max_leaf_size);
            node->r = construct_bvh(mid, end, max_leaf_size);
            return node;
        }

        bool BVHAccel::has_intersection(const Ray& ray, BVHNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.

              // base case: the node is leafeNode
            double t0, t1;
            // check if intersection exits in the current node
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }
            if (node->isLeaf()) { // if at leaf, check all primitives for interections
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    if ((*p)->has_intersection(ray)) {
                        return true;
                    }
                }
                return false;

            }
            else {
                total_isects++;
                bool hit = has_intersection(ray, node->r);
                return has_intersection(ray, node->l) || hit;
            }

            return false;



        }

        bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.

            double t0, t1;
            bool hit = false;
            // check if intersection exits in the current node
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }

            if (node->isLeaf()) { // if at leaf, check all primitives for interections
                for (auto p = node->start; p != node->end; p++) {

                    total_isects++;
                    if ((*p)->intersect(ray, i)) {
                        hit = true;
                    }
                }
                return hit;
            }
            else { // recursively check for interections
                total_isects++;
                hit = intersect(ray, i, node->l);
                return intersect(ray, i, node->r) || hit;
            }
            return false;

        }

    } // namespace SceneObjects
} // namespace CGL
