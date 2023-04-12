#pragma once
#include <iostream>
#include <memory>
#include <algorithm>
#include <vector>
#include <queue>
#include <deque> 
#include <cmath>
#include <memory>
#include <array>
#include <map>
#include <cmath>
#include <memory>
#include <pmp/SurfaceMesh.h>
#include <pmp/algorithms/SurfaceCurvature.h>
#include <pmp/algorithms/DifferentialGeometry.h>

namespace ortho {
	class topology {
	public:
		topology(const pmp::SurfaceMesh& mesh, std::vector<int>& vertex_vec, const pmp::EdgeProperty<float>& eweight_get);
		~topology() {};

	public:
		//id map
		void getmapper();
		double getWeight(int v1, int v2);
		//Dijkstra
		void Dijkstra(int v);
		std::deque<int> road(int v, int w);
		std::deque<int> DijkstraShortPath(int a, int b);

	private:
		int numVertices_ = 0;
		const long long int MAX_WEIGHT = RAND_MAX;
		std::vector<int> path_;
		std::vector<double> dist_;

		std::unordered_map<int, int> id_mapper_;
		std::vector<int>& vertex_vec_;
		const pmp::EdgeProperty<float>& eweight_get_;
		const pmp::SurfaceMesh& mesh_;
	};
}


