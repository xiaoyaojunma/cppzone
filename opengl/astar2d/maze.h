/*
 *  Copyright (c) 2012 Evgeny Proydakov <lord.tiran@gmail.com>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

#include <chrono>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

typedef float distance_t;

struct vertex_property
{
    size_t x;
    size_t y;
};

typedef boost::property<boost::edge_weight_t, distance_t> edge_property;

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, vertex_property, edge_property> grid;

typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;

class vertex_hash;

typedef std::vector<vertex_descriptor> vertex_vector;
typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;

typedef std::vector<std::string> name_vector;

///////////////////////////////////////////////////////////////////////////////

//
// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
//
class maze
{
public:
    enum maze_type {
        fixed,
        fixed_small,
        empty,
        random
    };

public:
    friend boost::shared_ptr<maze> fixed();
    friend boost::shared_ptr<maze> fixed_small();
    friend boost::shared_ptr<maze> random_maze(std::size_t x, std::size_t y);
    friend boost::shared_ptr<maze> empty_maze (std::size_t x, std::size_t y);

    maze(std::size_t width, std::size_t height);

    vertices_size_type length(std::size_t d) const;
    vertex_descriptor source();
    vertex_descriptor goal();

    bool solve();
    bool solved() const;
    const vertex_vector& get_way() const;
    const vertex_vector& get_solution() const;
    const vertex_set& get_barriers() const;
    bool has_barrier(vertex_descriptor u) const;

    size_t calc_vertex_index(size_t x, size_t y);

private:
    grid create_grid(std::size_t width, std::size_t height);
    filtered_grid create_barrier_grid();

private:
    size_t m_width;
    size_t m_height;

    grid          m_grid;
    filtered_grid m_barrier_grid;
    vertex_set    m_barriers;
    name_vector   m_name;

    std::vector<vertex_descriptor> m_solution;
    std::vector<vertex_descriptor> m_way;

    vertex_descriptor m_soure;
    vertex_descriptor m_goal;
};

// Generate a maze with a random assignment of barriers.
boost::shared_ptr<maze> fixed();
boost::shared_ptr<maze> fixed_small();
boost::shared_ptr<maze> random_maze(std::size_t x, std::size_t y);
boost::shared_ptr<maze> empty_maze(std::size_t x, std::size_t y);
