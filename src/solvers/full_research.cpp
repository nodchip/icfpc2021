#include "stdafx.h"
#include "contest_types.h"
#include "judge.h"
#include "timer.h"
#include "solver_registry.h"

#include <queue>
#include <fmt/core.h>
#include "visual_editor.h"
using vec = std::vector<integer>;
using mat = std::vector<vec>;

integer lower_limit(integer len_given, integer epsilon) {
	auto tmp = len_given * epsilon;
	tmp /= 1000000;
	return len_given - tmp;
}

integer upper_limit(integer len_given, integer epsilon) {
	auto tmp = len_given * epsilon;
	tmp /= 1000000;
	return len_given + tmp;
}



struct SVOI {
public:
	integer x_min;
	integer x_max;
	integer y_min; 
	integer y_max;

	SVOI() {
		x_min = INT64_MAX, x_max = INT64_MIN;
		y_min = INT64_MAX, y_max = INT64_MIN;
	}
	SVOI(const std::vector<Point>& points) {
		x_min = INT64_MAX, x_max = INT64_MIN;
		y_min = INT64_MAX, y_max = INT64_MIN;
		for (const auto [x, y] : points) {
			x_min = std::min(x, x_min);
			x_max = std::max(x, x_max);
			y_min = std::min(y, y_min);
			y_max = std::max(y, y_max);
			x_max++;
			y_max++;
		}
	}
	SVOI(const Point& point, integer length) {
		integer len = std::floor(std::sqrt(length)) + 1;
		x_min = get_x(point) - len;
		x_max = get_x(point) + len + 1;
		y_min = get_y(point) - len;
		y_max = get_y(point) + len + 1;
		
	}
	SVOI operator&(SVOI rhs) const {
		SVOI ret;
		ret.x_min = std::max(x_min, rhs.x_min);
		ret.x_max = std::min(x_max, rhs.x_max);
		ret.y_min = std::max(y_min, rhs.y_min);
		ret.y_max = std::min(y_max, rhs.y_max);
		return ret;
	}
	void operator&=(SVOI rhs) {
		x_min = std::max(x_min, rhs.x_min);
		x_max = std::min(x_max, rhs.x_max);
		y_min = std::max(y_min, rhs.y_min);
		y_max = std::min(y_max, rhs.y_max);
	}
	SVOI operator^(SVOI rhs) const {
		SVOI ret;
		ret.x_min = std::min(x_min, rhs.x_min);
		ret.x_max = std::max(x_max, rhs.x_max);
		ret.y_min = std::min(y_min, rhs.y_min);
		ret.y_max = std::max(y_max, rhs.y_max);
		return ret;
	}
	void operator^=(SVOI rhs) {
		x_min = std::min(x_min, rhs.x_min);
		x_max = std::max(x_max, rhs.x_max);
		y_min = std::min(y_min, rhs.y_min);
		y_max = std::max(y_max, rhs.y_max);
	}

	bool is_valid() {
		if (x_min >= x_max) return false;
		if (y_min >= y_max) return false;
		return true;
	}

};

bool is_point_valid(SProblemPtr problem, SSolutionPtr solution, int vertex, std::vector<int> decided_points, const mat& vertices_distances, Point& point) {
	for (auto e : decided_points) if (vertices_distances[e][vertex]) {
		auto length = distance2(solution->vertices[e], point);
		if (!tolerate(vertices_distances[e][vertex], length, problem->epsilon)) return false;
	}
	return true;
}
std::vector<Point> able_points(SProblemPtr problem, SSolutionPtr solution, int vertex, std::vector<int> decided_points, const mat& vertices_distances) {
	std::vector<Point> ret = {};
	SVOI voi(problem->hole_polygon);
	int last_e = -1;
	for (auto e : decided_points) if(vertices_distances[e][vertex]) {
		SVOI tmp(solution->vertices[e], upper_limit(vertices_distances[e][vertex],problem->epsilon));
		voi &= tmp;
		last_e = e;
	}
	CHECK(last_e >= 0);
	if (!voi.is_valid()) return ret;
	for (int y = voi.y_min; y < voi.y_max; y++) for(int x=voi.x_min; x< voi.x_max; x++){
		Point point = std::make_pair(x, y);
		if (is_point_valid(problem, solution, vertex, decided_points, vertices_distances, point)) ret.push_back(point);
	}
	return ret;
}


SSolutionPtr dfs_able_points(SProblemPtr problem, SSolutionPtr solution, std::vector<int> decided_points, std::vector<int> restriction_edges, const mat& vertices_distances, int V, long long& counter) {
	counter++;
	//if (counter % integer(1e4) == 0) LOG(INFO) << "counter is (able_points) " << counter;
	if (counter > 1e9) return nullptr;
	if (decided_points.size() == V) {
		auto judgement = judge(*problem, *solution);
		if (judgement.is_valid()) return solution;
		return nullptr;
	}
	int most_restricted_point = -1;
	int memo = -1;
	for (int i = 0; i < V; i++) if (memo < restriction_edges[i]) {
		memo = restriction_edges[i];
		most_restricted_point = i;
	}
	auto cands = able_points(problem, solution, most_restricted_point, decided_points, vertices_distances);
	if (cands.size() == 0) return nullptr;
	decided_points.push_back(most_restricted_point);
	for (int i = 0; i < V; i++) if (vertices_distances[most_restricted_point][i] && restriction_edges[i] >= 0) restriction_edges[i]++;
	restriction_edges[most_restricted_point] = -10000000;
	for (auto point : cands) {
		solution->vertices[most_restricted_point] = point;
		auto sol = dfs_able_points(problem, solution, decided_points, restriction_edges, vertices_distances, V, counter);
		if (sol) return sol;
	}
	return nullptr;


}

SSolutionPtr dfs_holes(SProblemPtr problem, const mat& hole_distances, const mat& vertices_distances, const std::vector<std::vector<double>>& upperlimit_distances, std::vector<int> v, int V, int H, Timer& timer, long long& counter) {
	constexpr int one_min = 1000 * 60;
	counter++;
	//if(counter % integer(1e4) == 0) LOG(INFO) << "counter is (holes)" << counter;
	if (counter > 1e9) return nullptr;
	//LOG(INFO) << "v.size is : " << v.size();
	//if (timer.elapsed_ms() > one_min) return nullptr;
 	if (v.size() >= H) {
		std::vector<int> restriction_edges(V,0);
		for (auto e : v) for (int i = 0; i < V; i++) if (vertices_distances[i][e]) restriction_edges[i]++;
		for (auto e : v) restriction_edges[e] = -100000;
		std::vector<Point> points(V, std::make_pair(0,0));
		for (int i = 0; i < H; i++) {
			auto e = v[i];
			points[e] = problem->hole_polygon[i];
		}
		SSolutionPtr solution(new SSolution(points));
		return dfs_able_points(problem, solution, v, restriction_edges, vertices_distances, V, counter);

	}
	else {
		for (int x = 0; x < V; x++) {

			v.push_back(x);
			bool okay = true;

			for (int i = 0; i < v.size() - 1; i++) {
				int y = v[i];
				if (y == x) {
					okay = false;
					break;
				}
				auto ver_dist = vertices_distances[x][y];
				auto uplimit_dist = upperlimit_distances[x][y];
				auto hole_dist = hole_distances[i][v.size() - 1];
				//LOG(INFO) << fmt::format("Problem  : {}, {}, {}", x,y);
				if (ver_dist && !tolerate(ver_dist, hole_dist, problem->epsilon)) okay = false;
				if (uplimit_dist + 1 < std::sqrt(hole_dist)) okay = false;
				if (!okay) break;
			}
			//counter++;
			if (okay) {
				//LOG(INFO) << "dfs_holes:: v is ";
				//for (auto e : v) LOG(INFO) << e;
				auto sol = dfs_holes(problem, hole_distances, vertices_distances, upperlimit_distances, v, V, H, timer, counter);
				if (sol) return sol;
			}
			v.pop_back();
		}
	}
	return nullptr;
}





#if 1
bool full_research(SProblemPtr problem, SSolutionPtr& solution, Timer& timer) {
	int V = problem->vertices.size();
	int H = problem->hole_polygon.size();

	
	constexpr double BIGDOUBLE = 1e18;

	mat vertices_distances(V, vec(V, 0));
	std::vector<std::vector<double>> upperlimit_distances(V, std::vector<double>(V, BIGDOUBLE));

	for (const auto& e : problem->edges) {
		vertices_distances[e.first][e.second] = distance2(problem->vertices[e.first], problem->vertices[e.second]);
		vertices_distances[e.second][e.first] = vertices_distances[e.first][e.second];
		upperlimit_distances[e.first][e.second] = std::sqrt(upper_limit(distance2(problem->vertices[e.first], problem->vertices[e.second]), problem->epsilon));
		upperlimit_distances[e.second][e.first] = upperlimit_distances[e.first][e.second];
	}

	for (int k = 0; k < V; k++) for (int i = 0; i < V; i++) for (int j = 0; j < V; j++) {
		upperlimit_distances[i][j] = std::min(upperlimit_distances[i][j], upperlimit_distances[i][k] + upperlimit_distances[k][j]);
	}

	mat hole_distances(H, vec(H, 0));
	for (int i = 0; i < H; i++) for (int j = 0; j < H; j++) if (i != j) hole_distances[i][j] = distance2(problem->hole_polygon[i], problem->hole_polygon[j]);
	long long counter = 0;
	std::vector<std::vector<int>> ret;
	std::vector<int> v = {};
	auto sol = dfs_holes(problem, hole_distances, vertices_distances, upperlimit_distances, v, V, H, timer, counter);
	if (counter >= 1e9) LOG(INFO) << "counter over";
	if (sol) {
		solution = sol;
		return true;
	}
	return false;

}



class FullResearch : public SolverBase {
public:
	FullResearch() { }
	SolverOutputs solve(const SolverArguments& args) override {
		SolverOutputs ret;

		Timer timer;
		ret.solution = std::make_shared<SSolution>();
		ret.solution->vertices = args.problem->vertices;
		full_research(args.problem, ret.solution, timer);

		// dummy.
		//std::this_thread::sleep_for(std::chrono::milliseconds(500));

		return ret;
	}
};

REGISTER_SOLVER("FullResearch", FullResearch);
#endif