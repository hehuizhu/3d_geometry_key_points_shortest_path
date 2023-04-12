#include "shortest_path_wrapper.h"
#include <fstream>

namespace ortho {
	ShortestPathClass::ShortestPathClass(pmp::SurfaceMesh& mesh,bool isweight) :mesh_(mesh) {
		isweight_ = isweight;
		compute_curvature();
		get_weight();
	}

	void ShortestPathClass::get_weight() {
		auto points_all = mesh_.get_vertex_property<pmp::Point>("v:point");
		auto min_curvature = mesh_.get_vertex_property<float>("v:curv_min");
		auto max_curvature = mesh_.get_vertex_property<float>("v:curv_max");
		if (!mesh_.has_edge_property("e:weight")) {
			auto eweight = mesh_.add_edge_property<float>("e:weight");
			for (const auto& e : mesh_.edges()) {
				pmp::Point v = points_all[mesh_.vertex(e, 0)] - points_all[mesh_.vertex(e, 1)];
				float cur_v = 0.;
				if (CurvType_ == "Mean") {
					//Mean curvature
					cur_v = 0.5f * (0.5f * (std::fabs(min_curvature[mesh_.vertex(e, 0)])
						+ std::fabs(max_curvature[mesh_.vertex(e, 0)]))
						+ 0.5f * (std::fabs(min_curvature[mesh_.vertex(e, 1)])
							+ std::fabs(max_curvature[mesh_.vertex(e, 1)])));
				}

				if (CurvType_ == "RMS") {
					//RMS
					float cur_v1 = std::sqrt(0.5f * (std::pow(min_curvature[mesh_.vertex(e, 0)], 2) + std::pow(max_curvature[mesh_.vertex(e, 0)], 2)));
					float cur_v2 = std::sqrt(0.5f * (std::pow(min_curvature[mesh_.vertex(e, 1)], 2) + std::pow(max_curvature[mesh_.vertex(e, 1)], 2)));

					cur_v = 0.5f * (cur_v1 + cur_v2);
				}

				if (CurvType_ == "MAXABS") {
					//MAXABS
					cur_v = 0.5f * (std::max(std::fabs(min_curvature[mesh_.vertex(e, 0)]), std::fabs(max_curvature[mesh_.vertex(e, 0)]))
						+ std::max(std::fabs(min_curvature[mesh_.vertex(e, 1)]), std::fabs(max_curvature[mesh_.vertex(e, 1)])));
				}
				if(isweight_)
					eweight[e] = pmp::norm(v) / (std::pow(cur_v + 0.0001f, 0.4)+1);
				else
					eweight[e] = pmp::norm(v);
					//eweight[e] = pmp::norm(v) / (std::pow(cur_v + 0.0001f, 0.3) + 1);
			}
		}
	}

	std::vector<int> ShortestPathClass::getShortestPath(int v_start, int v_end) {
		auto min_curvature = mesh_.get_vertex_property<float>("v:curv_min");
		auto max_curvature = mesh_.get_vertex_property<float>("v:curv_max");
		auto eweight_get = mesh_.get_edge_property<float>("e:weight");
		auto points_all = mesh_.get_vertex_property<pmp::Point>("v:point");

		pmp::Point p1 = points_all[pmp::Vertex(v_start)], p2 = points_all[pmp::Vertex(v_end)];

		//set radius
		float d1 = pmp::norm(p1 - p2) / 2.f + 0.03f;
		pmp::Point center_p = (p1 + p2) / 2.f;

		//add select part
		std::vector<int> ver_vec;
		auto vselected = mesh_.add_vertex_property<bool>("v:selected", false);
		for (const auto& v : mesh_.vertices()) {
			pmp::Point v1 = points_all[v] - center_p;
			float d2 = pmp::norm(v1) / 2.f;
			if (d2 < d1) {
				vselected[v] = true;
				ver_vec.emplace_back(v.idx());
			}
		}

		ortho::topology dij_sp(mesh_, ver_vec, eweight_get);
		std::deque<int> shorest_road = dij_sp.DijkstraShortPath(v_start, v_end);
		mesh_.remove_vertex_property(vselected);
		return std::vector<int>(shorest_road.begin(), shorest_road.end());
	}

	void ShortestPathClass::compute_curvature() {
		auto min_curvature = mesh_.add_vertex_property<float>("v:curv_min", 10000.);
		auto max_curvature = mesh_.add_vertex_property<float>("v:curv_max", 10000.);

		float kmin, kmax, mean, gauss;
		float area, sum_angles;
		float weight, sum_weights;
		pmp::Point p0, p1, p2, laplace;

		//std::ofstream outfile("E:/ortho-key-points-shortest-path/examples/curvature.txt");

		auto cotan = mesh_.add_edge_property<double>("curv:cotan");
		for (auto e : mesh_.edges())
			cotan[e] = pmp::cotan_weight(mesh_, e);

		for (auto v : mesh_.vertices()) {
			kmin = kmax = 0.0;
			if (!mesh_.is_isolated(v) && !mesh_.is_boundary(v)) {
				laplace = pmp::Point(0.0);
				sum_weights = 0.0;
				sum_angles = 0.0;
				p0 = mesh_.position(v);

				area = pmp::voronoi_area(mesh_, v);
				for (auto vh : mesh_.halfedges(v)) {
					p1 = mesh_.position(mesh_.to_vertex(vh));
					p2 = mesh_.position(mesh_.to_vertex(mesh_.ccw_rotated_halfedge(vh)));

					weight = cotan[mesh_.edge(vh)];
					sum_weights += weight;
					laplace += weight * p1;

					p1 -= p0;
					p1.normalize();
					p2 -= p0;
					p2.normalize();
					sum_angles += std::acos(pmp::clamp_cos(dot(p1, p2)));
				}

				laplace -= sum_weights * mesh_.position(v);
				laplace /= 2.0f * area;

				mean = 0.5f * pmp::norm(laplace);
				gauss = (2.0 * M_PI - sum_angles) / area;

				const float s = std::sqrt(std::max(float(0.0), mean * mean - gauss));
				kmin = mean - s;
				kmax = mean + s;
			}
			min_curvature[v] = kmin;
			max_curvature[v] = kmax;

			//output curvature
			//outfile << v.idx() << ' ' << kmin << ' ' << kmax << '\n';
		}

		//outfile.close();

		//clean properties
		mesh_.remove_edge_property(cotan);
	}
}
