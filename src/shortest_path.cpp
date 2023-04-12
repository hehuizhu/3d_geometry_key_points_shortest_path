#include "shortest_path.h"

namespace ortho {
	topology::topology(const pmp::SurfaceMesh& mesh, std::vector<int>& vertex_vec, const pmp::EdgeProperty<float>& eweight_get) :
		mesh_(mesh), vertex_vec_(vertex_vec), eweight_get_(eweight_get)	{
		numVertices_ = vertex_vec_.size();
		getmapper();
	}

	void topology::getmapper() {
		for (int i = 0; i < vertex_vec_.size(); i++)
			id_mapper_[vertex_vec_[i]] = i;
	}

	//input id 0,1,2.... 
	double topology::getWeight(int v1, int v2) {
		if (v1 >= numVertices_ || v2 >= numVertices_ || v1 == v2) 
			return 	MAX_WEIGHT;
		
		pmp::Edge e = mesh_.find_edge(pmp::Vertex(vertex_vec_[v1]), pmp::Vertex(vertex_vec_[v2]));
		return e.idx() != PMP_MAX_INDEX ? eweight_get_[e] : MAX_WEIGHT;
	}

	void topology::Dijkstra(int v) {
		int k = 0;
		std::vector<bool> s(numVertices_);
		std::vector<int> path(numVertices_);
		std::vector<double> dist(numVertices_);

		// initialize
		for (int i = 0; i < numVertices_; i++) {
			s[i] = false;
			dist[i] = getWeight(v, i);
			path[i] = (dist[i] < MAX_WEIGHT || v == i) ? v : -1;
		}

		s[v] = true;
		dist[v] = 0;
		for (int i = 0; i < numVertices_; i++) {
			double min = MAX_WEIGHT;
			for (int j = 0; j < numVertices_; j++) {
				if (!s[j] && dist[j] < min) {
					k = j;
					min = dist[j];
				}
			}
			s[k] = true;
			for (int w = 0; w < numVertices_; w++) {
				if (!s[w] && dist[w] > dist[k] + getWeight(k, w) && getWeight(k, w) != MAX_WEIGHT) {
					dist[w] = dist[k] + getWeight(k, w);
					path[w] = k;
				}
			}
		}
		path_ = path;
		dist_ = dist;
	}

	std::deque<int> topology::road(int v, int w) {
		int current = w;
		std::deque<int> Road;
		Road.push_front(vertex_vec_[w]);
		while (current != v) {
			int previous = path_[current];
			Road.push_front(vertex_vec_[previous]);
			current = previous;
		}

		return Road;
	}

	//Dijkstra algorithm  --gain the shortest path between a and b
	std::deque<int> topology::DijkstraShortPath(int a, int b) {
		int v = id_mapper_[a];
		Dijkstra(v);
		int w = id_mapper_[b];
		return road(v, w);
	}
}


