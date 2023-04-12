#include "shortest_path.h"

namespace ortho {
	class ShortestPathClass {
	public:
		ShortestPathClass(pmp::SurfaceMesh& mesh,bool isweight=true);
		~ShortestPathClass() { 
			auto min_curvature1 = mesh_.get_vertex_property<float>("v:curv_min");
			auto max_curvature1 = mesh_.get_vertex_property<float>("v:curv_max");
			mesh_.remove_vertex_property(min_curvature1);
			mesh_.remove_vertex_property(max_curvature1);
			auto eweight1 = mesh_.get_edge_property<float>("e:weight");
			mesh_.remove_edge_property(eweight1);
		};
	public:
		void get_weight();
		void compute_curvature();
		std::vector<int> getShortestPath(int v_start, int v_end);

	private:
		const long long int MAX_WEIGHT = RAND_MAX;
		std::string CurvType_ = "RMS"; //"MEAN" "RMS" "MAXABS"
		pmp::SurfaceMesh& mesh_;
		bool isweight_;
	};
}
