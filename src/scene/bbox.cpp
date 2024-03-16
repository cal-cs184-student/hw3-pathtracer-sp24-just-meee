#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

	bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

		// TODO (Part 2.2):
		// Implement ray - bounding box intersection test
		// If the ray intersected the bouding box within the range given by
		// t0, t1, update t0 and t1 with the new intersection times.
		double t_min = -std::numeric_limits<int>::min();;
		double t_max = std::numeric_limits<double>::max();

		for (int i = 0; i < 3; i++) {
			// find intersections
			double inverse_d = 1.0 / r.d[i];
			double t_close = (min[i] - r.o[i]) * inverse_d;
			double t_far = (max[i] - r.o[i]) * inverse_d;
			if (t_close > t_far) {
				std::swap(t_close, t_far);
			}

			// update min and max
			t_min = std::max(t_min, t_close);
			t_max = std::min(t_max, t_far);
			// check if the interval is valid
			if (t_min > t_max) {
				return false;
			}
		}
		t0 = t_min;
		t1 = t_max;
		return true;

	}

	void BBox::draw(Color c, float alpha) const {

		glColor4f(c.r, c.g, c.b, alpha);

		// top
		glBegin(GL_LINE_STRIP);
		glVertex3d(max.x, max.y, max.z);
		glVertex3d(max.x, max.y, min.z);
		glVertex3d(min.x, max.y, min.z);
		glVertex3d(min.x, max.y, max.z);
		glVertex3d(max.x, max.y, max.z);
		glEnd();

		// bottom
		glBegin(GL_LINE_STRIP);
		glVertex3d(min.x, min.y, min.z);
		glVertex3d(min.x, min.y, max.z);
		glVertex3d(max.x, min.y, max.z);
		glVertex3d(max.x, min.y, min.z);
		glVertex3d(min.x, min.y, min.z);
		glEnd();

		// side
		glBegin(GL_LINES);
		glVertex3d(max.x, max.y, max.z);
		glVertex3d(max.x, min.y, max.z);
		glVertex3d(max.x, max.y, min.z);
		glVertex3d(max.x, min.y, min.z);
		glVertex3d(min.x, max.y, min.z);
		glVertex3d(min.x, min.y, min.z);
		glVertex3d(min.x, max.y, max.z);
		glVertex3d(min.x, min.y, max.z);
		glEnd();

	}

	std::ostream& operator<<(std::ostream& os, const BBox& b) {
		return os << "BBOX(" << b.min << ", " << b.max << ")";
	}

} // namespace CGL
