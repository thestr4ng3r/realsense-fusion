
#ifndef _IMPLICIT_H
#define _IMPLICIT_H

class Sphere
{
	public:
		Sphere(float radius)
		{
			this->radius = radius;
		}

		~Sphere(){}

		Sphere(const float radius, const float x_center, const float y_center, const float z_center)
		{
			this->radius = radius;
			this->x_center = x_center;
			this->y_center = y_center;
			this->z_center = z_center;
		}

		float sdf(float x, float y, float z) {
			x = x - x_center;
			y = y - y_center;
			z = z - z_center;
			return sqrt(x*x + y*y + z*z) - radius;
		}

	private:
		float radius;
		float x_center;
		float y_center;
		float z_center;
};
#endif // !_H_IMPLICIT
