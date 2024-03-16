#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
    namespace SceneObjects {

        Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) {
            p1 = mesh->positions[v1];
            p2 = mesh->positions[v2];
            p3 = mesh->positions[v3];
            n1 = mesh->normals[v1];
            n2 = mesh->normals[v2];
            n3 = mesh->normals[v3];
            bbox = BBox(p1);
            bbox.expand(p2);
            bbox.expand(p3);

            bsdf = mesh->get_bsdf();
        }

        BBox Triangle::get_bbox() const { return bbox; }

        bool Triangle::has_intersection(const Ray& r) const {
            // Part 1, Task 3: implement ray-triangle intersection
            // The difference between this function and the next function is that the next
            // function records the "intersection" while this function only tests whether
            // there is a intersection.

              //Vector3D N = cross((p2 - p1), (p3 - p1)).unit();
              //double t = dot(N, (p1 - r.o)) / dot(N, r.d);
              //if (t < r.min_t || t > r.max_t) {
              //    return false;
              //}
              //// get the intercetion point
              //Vector3D p = r.o + t * r.d;

              //// get the barycentric coordinates and check if the point is inside the triangle
              //Vector3D u = barycentric_coordinates(p);
              //if (u.x < 0 || u.y < 0 || u.z < 0) {
              //    return false;
              //}
              //return true;
            Intersection* isect;
            return intersect(r, isect);

        }

        bool Triangle::intersect(const Ray& r, Intersection* isect) const {
            // Part 1, Task 3:
            // implement ray-triangle intersection. When an intersection takes
            // place, the Intersection data should be updated accordingly
            //
            // The algrithm below is generated by AI.
            // I used AI by direcly asking for triangle intersection test 
            // The algrithm is much more effiecient as it does many of the arithmtic operations by matrices.
            // and has fewers operations. I learned a new way to do triagnle testing by using the AI tool.
              // Compute vectors representing edges and the ray origin relative to the triangle vertices
            Vector3D edge1 = p2 - p1;
            Vector3D edge2 = p3 - p1;
            Vector3D ray_origin_offset = r.o - p1;

            // Compute vectors used for barycentric coordinate calculation
            Vector3D cross1 = cross(r.d, edge2);
            Vector3D cross2 = cross(ray_origin_offset, edge1);

            // Calculate barycentric coordinates
            double denominator = dot(cross1, edge1);

            // Calculate each component of the result vector separately
            double alpha_numerator = dot(cross2, edge2);
            double beta_numerator = dot(cross1, ray_origin_offset);
            double gamma_numerator = dot(cross2, r.d);

            // Construct the result vector using the calculated components and denominator
            Vector3D result = (1.0 / denominator) * Vector3D(alpha_numerator, beta_numerator, gamma_numerator);

            double t = result[0];
            double alpha = result[1];
            double beta = result[2];
            double gamma = 1.0 - alpha - beta;

            // Check if the intersection is valid and if t is within range
            if (alpha < 0 || beta < 0 || gamma < 0 || t < r.min_t || t > r.max_t)
                return false;

            // Update the maximum intersection distance
            r.max_t = t;

            // Populate the intersection struct
            isect->t = t;
            isect->n = n1 * alpha + n2 * beta + n3 * gamma;
            isect->primitive = this;
            isect->bsdf = get_bsdf();

            return true;

        }

        /*Vector3D Triangle::barycentric_coordinates(const Vector3D& p) const {
            double sample_x = p.x;
            double sample_y = p.y;
            double x0 = p1.x;
            double x1 = p2.x;
            double x2 = p3.x;
            double y0 = p1.y;
            double y1 = p2.y;
            double y2 = p3.y;

            double a = (-(sample_x - x1) * (y2 - y1) + (sample_y - y1) * (x2 - x1)) /
                (-(x0 - x1) * (y2 - y1) + (y0 - y1) * (x2 - x1));
            double b = (-(sample_x - x2) * (y0 - y2) + (sample_y - y2) * (x0 - x2)) /
                (-(x1 - x2) * (y0 - y2) + (y1 - y2) * (x0 - x2));
            double r = 1 - a - b;

            return Vector3D(a, b, r);
        }*/

        void Triangle::draw(const Color& c, float alpha) const {
            glColor4f(c.r, c.g, c.b, alpha);
            glBegin(GL_TRIANGLES);
            glVertex3d(p1.x, p1.y, p1.z);
            glVertex3d(p2.x, p2.y, p2.z);
            glVertex3d(p3.x, p3.y, p3.z);
            glEnd();
        }

        void Triangle::drawOutline(const Color& c, float alpha) const {
            glColor4f(c.r, c.g, c.b, alpha);
            glBegin(GL_LINE_LOOP);
            glVertex3d(p1.x, p1.y, p1.z);
            glVertex3d(p2.x, p2.y, p2.z);
            glVertex3d(p3.x, p3.y, p3.z);
            glEnd();
        }

    } // namespace SceneObjects
} // namespace CGL
