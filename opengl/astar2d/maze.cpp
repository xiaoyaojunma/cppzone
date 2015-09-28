#include "maze.h"

boost::mt19937 random_generator;

// Exception thrown when the goal vertex is found
struct found_goal {};

///////////////////////////////////////////////////////////////////////////////

// A hash function for vertices.
class vertex_hash : std::unary_function<vertex_descriptor, std::size_t>
{
public:
    vertex_hash(const grid& graph) : m_graph(graph) {}

    std::size_t operator()(const vertex_descriptor& v) const
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, m_graph[v].x);
        boost::hash_combine(seed, m_graph[v].y);
        return seed;
    }

private:
    const grid& m_graph;
};

///////////////////////////////////////////////////////////////////////////////

//
// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
//
template<class graph_t>
class euclidean_heuristic : public boost::astar_heuristic<graph_t, distance_t>
{
public:
    euclidean_heuristic(const graph_t& graph, const vertex_descriptor& goal) : m_graph(graph), m_goal(goal) {}

    distance_t operator() (vertex_descriptor v) {
        double x = std::abs((m_graph[m_goal].x - m_graph[v].x));
        double y = std::abs((m_graph[m_goal].y - m_graph[v].y));
        return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    }

private:
    const graph_t& m_graph;
    vertex_descriptor m_goal;
};

///////////////////////////////////////////////////////////////////////////////

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor : public boost::default_astar_visitor
{
    astar_goal_visitor(const vertex_descriptor& goal) : m_goal(goal) {}

    void examine_vertex(const vertex_descriptor& u, const grid& graph) {
        if (u == m_goal) {
            throw found_goal();
        }
    }

private:
    vertex_descriptor m_goal;
};

///////////////////////////////////////////////////////////////////////////////

maze::maze(std::size_t width, std::size_t height) :
    m_width(width),
    m_height(height),
    m_grid(create_grid(x, y)),
    m_barrier_grid(create_barrier_grid())
{
}

vertices_size_type maze::length(std::size_t d) const
{
    return m_grid.length(d);
}

vertex_descriptor maze::source() const
{
    return m_soure;
}

vertex_descriptor maze::goal() const
{
    return m_goal;
}

bool maze::solve()
{
    const size_t solution_assessment_size = m_width + m_height;
    vertex_hash hash(m_grid);

    // The predecessor map is a vertex-to-vertex mapping.
    typedef boost::unordered_map<vertex_descriptor, vertex_descriptor, vertex_hash> pred_map;
    pred_map predecessor(solution_assessment_size, hash);
    boost::associative_property_map<pred_map> pred_pmap(predecessor);

    // The distance map is a vertex-to-distance mapping.
    typedef boost::unordered_map<vertex_descriptor, distance_t, vertex_hash> dist_map;
    dist_map distance(solution_assessment_size, hash);
    boost::associative_property_map<dist_map> dist_pmap(distance);

    euclidean_heuristic heuristic(m_grid, goal);
    astar_goal_visitor  visitor  (goal);

    bool result = false;
    const int iters = 10;

    std::chrono::high_resolution_clock clock;
    auto start = clock.now();

    for(int i = 0; i < iters; i++) {
        solution.clear();
        predecessor.clear();
        distance.clear();
        try {
            boost::astar_search(m_grid, source, heuristic,
                                boost::visitor(visitor).
                                predecessor_map(pred_pmap).
                                distance_map(dist_pmap)
                                );
        }
        catch(const found_goal& fg) {
            // Walk backwards from the goal through the predecessor chain adding
            // vertices to the solution path.
            for (vertex_descriptor u = goal; u != source; u = predecessor[u]) {
                solution.push_back(u);
            }
            solution.push_back(source);
            std::reverse(solution.begin(), solution.end());
            result = true;
        }
    }
    auto end = clock.now();

    distance_t dist = distance[goal];
    std::cout << "solve. result: " << result << " size: " << dist << " process time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / iters << " us"
              << std::endl;

    return result;
}

bool maze::solved() const
{
    return !m_solution.empty();
}

const vertex_vector& maze::get_way() const
{
    return m_way;
}

const vertex_vector& maze::get_solution() const
{
    return m_solution;
}

const vertex_set& maze::get_barriers() const
{
    return m_barriers;
}

bool maze::has_barrier(vertex_descriptor u) const
{
    return m_barriers.find(u) != m_barriers.end();
}

size_t maze::calc_vertex_index(size_t x, size_t y)
{
    return x + y * m_width;
}

grid maze::create_grid(std::size_t width, std::size_t height)
{
    const size_t size = width * height;
    m_name.resize(size);

    // init horizontal edge
    const distance_t horizontal_size = 1;
    for(size_t x = 0; x < width - 1; x++) {
        for(size_t y = 0; y < height; y++) {
            const size_t index = calc_vertex_index(x, y);
            boost::add_edge(index, index + 1, horizontal_size, m_grid);
        }
    }

    // init vertical edge
    const distance_t vertical_size = 1;
    for(size_t x = 0; x < width; x++) {
        for(size_t y = 0; y < height - 1; y++) {
            const size_t index = calc_vertex_index(x, y);
            boost::add_edge(index, index + width, vertical_size, m_grid);
        }
    }

    // init diagonal edge
    const distance_t diagonal_size = std::sqrt(2);
    for(size_t x = 0; x < width - 1; x++) {
        for(size_t y = 0; y < height - 1; y++) {
            const size_t index = calc_vertex_index(x, y);
            boost::add_edge(index, index + width + 1, diagonal_size, m_grid);
            boost::add_edge(index + 1, index + width, diagonal_size, m_grid);
        }
    }

    // fill name and vertex descriptin
    for(size_t x = 0; x < width; x++) {
        for(size_t y = 0; y < height; y++) {
            const size_t index = calc_vertex_index(x, y);
            vertex_descriptor descriptor(index);
            m_grid[descriptor].x = x;
            m_grid[descriptor].y = y;
            std::stringstream sstream;
            sstream << "(" << x << ", " << y << ")";
            m_name[index] = sstream.str();
        }
    }
}

filtered_grid maze::create_barrier_grid()
{
    return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
}

// Return a random integer in the interval [a, b].
std::size_t random_int(std::size_t a, std::size_t b)
{
    if (b < a) {
        b = a;
    }
    boost::uniform_int<> dist(a, b);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> >
            generate(random_generator, dist);
    return generate();
}

// Generate a maze with a random assignment of barriers.
boost::shared_ptr<maze> random_maze(std::size_t width, std::size_t height)
{
    boost::shared_ptr<maze> m = boost::shared_ptr<maze>(new maze(width, height));

    vertex_descriptor s = m->calc_vertex_index( 0, 0 );
    vertex_descriptor g = m->calc_vertex_index( width - 1, height -1 );
    m->m_soure = s;
    m->m_goal  = g;

    vertices_size_type n = num_vertices(m->m_grid);

    // One quarter of the cells in the maze should be barriers.
    int barriers = pow(n, 0.75);
    while (barriers > 0) {
        // Choose horizontal or vertical direction.
        std::size_t direction = random_int(0, 1);
        // Walls range up to one quarter the dimension length in this direction.
        vertices_size_type wall = random_int(1, m->length(direction) / 4);
        // Create the wall while decrementing the total barrier count.
        vertex_descriptor u = vertex(random_int(0, n - 1), m->m_grid);
        while (wall) {
            // Start and goal spaces should never be barriers.
            if (u != s && u != g) {
                wall--;
                if (!m->has_barrier(u)) {
                    m->m_barriers.insert(u);
                    barriers--;
                }
            }
            vertex_descriptor v = m->m_grid.next(u, direction);
            // Stop creating this wall if we reached the maze's edge.
            if (u == v) {
                break;
            }
            u = v;
        }
    }
    return m;
}

boost::shared_ptr<maze> empty_maze(std::size_t x, std::size_t y)
{
    boost::shared_ptr<maze> m = boost::shared_ptr<maze>(new maze(x, y));

    vertex_descriptor s = m->calc_vertex_index( 0, 0 );
    vertex_descriptor g = ( x - 1, y -1 );
    m->m_soure = s;
    m->m_goal  = g;

    return m;
}

boost::shared_ptr<maze> fixed()
{
    boost::shared_ptr<maze> m = boost::shared_ptr<maze>(new maze(9, 9));

    vertex_descriptor u1 = m->calc_vertex_index(4, 2);
    vertex_descriptor u2 = m->calc_vertex_index(4, 3);
    vertex_descriptor u3 = m->calc_vertex_index(4, 4);
    vertex_descriptor u4 = m->calc_vertex_index(4, 5);

    m->m_barriers.insert(u1);
    m->m_barriers.insert(u2);
    m->m_barriers.insert(u3);
    m->m_barriers.insert(u4);

    vertex_descriptor source = m->calc_vertex_index(2, 4);
    m->m_soure = source;

    vertex_descriptor goal = m->calc_vertex_index(6, 4);
    m->m_goal = goal;

    return m;
}

boost::shared_ptr<maze> fixed_small()
{
    boost::shared_ptr<maze> m = boost::shared_ptr<maze>(new maze(3, 3));

    vertex_descriptor u1 = m->calc_vertex_index(1, 0);
    vertex_descriptor u2 = m->calc_vertex_index(1, 2);

    m->m_barriers.insert(u1);
    m->m_barriers.insert(u2);

    vertex_descriptor source = m->calc_vertex_index(0, 0);
    m->m_soure = source;

    vertex_descriptor goal = m->calc_vertex_index(2, 0);
    m->m_goal = goal;

    return m;
}
