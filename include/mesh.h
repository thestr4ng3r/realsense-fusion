
#ifndef MESH_H
#define MESH_H

#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Vector3f Vertex;

struct Triangle
{
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;
	int color[3];
	Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2, int _color[3]) :
		idx0(_idx0), idx1(_idx1), idx2(_idx2), color{ _color[0], _color[1], _color[2] }
	{}
};

class Mesh
{
public:

	void Clear()
	{
		m_vertices.clear();
		m_triangles.clear();
	}

	unsigned int AddVertex(Vertex& vertex)
	{
		unsigned int vId = (unsigned int)m_vertices.size();
		m_vertices.push_back(vertex);
		return vId;
	}

	unsigned int AddFace(unsigned int idx0, unsigned int idx1, unsigned int idx2, int color[3])
	{
		unsigned int fId = (unsigned int)m_triangles.size();
		Triangle triangle(idx0, idx1, idx2, color);
		m_triangles.push_back(triangle);
		return fId;
	}

	std::vector<Vertex>& GetVertices()
	{
		return m_vertices;
	}

	std::vector<Triangle>& GetTriangles()
	{
		return m_triangles;
	}

	bool WriteMesh(const std::string& filename, bool color_active)
	{
		// Write off file
		std::ofstream outFile(filename);
		if (!outFile.is_open()) return false;

		// write header
		outFile << "OFF" << std::endl;
		outFile << m_vertices.size() << " " << m_triangles.size() << " 0" << std::endl;

		// save vertices
		for (unsigned int i = 0; i < m_vertices.size(); i++)
		{
			outFile << m_vertices[i].x() << " " << m_vertices[i].y() << " " << m_vertices[i].z() << std::endl;
		}

		if (color_active) 
		{
			// save faces
			for (unsigned int i = 0; i < m_triangles.size(); i++)
			{
				outFile << "3 " << m_triangles[i].idx0 << " " << m_triangles[i].idx1 << " " << m_triangles[i].idx2 <<
					" " << m_triangles[i].color[0] << " " << m_triangles[i].color[1] << " " << m_triangles[i].color[2] << std::endl;
			}
		}
		else
		{
			// save faces
			for (unsigned int i = 0; i < m_triangles.size(); i++)
			{
				outFile << "3 " << m_triangles[i].idx0 << " " << m_triangles[i].idx1 << " " << m_triangles[i].idx2 << std::endl;
			}
		}
		

		// close file
		outFile.close();

		return true;
	}

private:
	std::vector<Vertex> m_vertices;
	std::vector<Triangle> m_triangles;
};

class PointCloud
{
private:
	std::vector<Eigen::Vector3f> m_points;
	std::vector<Eigen::Vector3f> m_normals;

};

#endif // MESH_H
