/*
 *  Copyright (c) 2011 Evgeny Proydakov <lord.tiran@gmail.com>
 */

#include <map>
#include <list>
#include <vector>
#include <iostream>
#include <algorithm>

//#define TEST // DEFINE FOR BUILD TEST EXAMPLE

typedef long long length_type;
typedef int index_type;

class node
{
public:
	node() :
	    parent(NULL), 
	    length_to_the_parent(0), 
	    parent_name(0)
	{
	}
	
	bool is_head() const {
		return parent == NULL;
	}
	
	friend bool operator==(const node& a_node, const node& b_node) {
		return a_node.parent == b_node.parent &&
				a_node.length_to_the_parent == b_node.length_to_the_parent &&
				a_node.parent_name == b_node.parent_name;
	}
	
	node* parent;
	index_type length_to_the_parent;
	index_type parent_name;
};
typedef std::pair<index_type, node> graph_element;
typedef std::map<index_type, node> graph;

typedef std::pair<index_type, length_type> way_element;
typedef std::map<index_type, length_type> way;

typedef std::pair<index_type, index_type> task_pair;
typedef std::list<task_pair> task;

void read_input_data(graph& data);
void read_input_task(task& data);

length_type search_distances_on_the_graph(const graph& data, const task_pair& pair);

void create_example_graph(graph& data);
void create_example_task(task& data);

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    
    graph my_graph;
#ifndef TEST
    read_input_data(my_graph);
#else
    create_example_graph(my_graph);
#endif
    
    task my_task;
#ifndef TEST
    read_input_task(my_task);
#else
    create_example_task(my_task);
#endif
    
    task::const_iterator endIt = my_task.end();
    for(task::const_iterator it = my_task.begin(); it != endIt; ++it) {
#ifdef TEST
        std::cout << "PAIR:  " << it->first << " " << it->second << std::endl;
        std::cout << "LENGTH:  ";
#endif
        std::cout << search_distances_on_the_graph(my_graph, *it) << std::endl;
    }
    return 0;
}

void read_input_data(graph& data)
{
#ifdef TEST
	std::cout << "Enter number of graph vertices: ";
#endif
	int input_size = 0;
	std::cin >> input_size;
	
	index_type graph_parent_name;
	index_type graph_node_name;
	length_type length_to_the_parent;
	
	node my_node;
	index_type node_name = 1;
	data.insert(graph_element(node_name, my_node));
	
	for(int i = 2; i <= input_size; ++i) {
#ifdef TEST
		std::cout << "Enter data graph for the element " << i << " (parent, element, length_to_the_parent):  ";
#endif
		std::cin >> graph_parent_name;
		std::cin >> graph_node_name;
		std::cin >> length_to_the_parent;
		
		my_node.length_to_the_parent = length_to_the_parent;
		my_node.parent_name = graph_parent_name;
		my_node.parent = &data.find(graph_parent_name)->second;
		
		data.insert(graph_element(graph_node_name, my_node));
	}	
}

void read_input_task(task& data)
{
#ifdef TEST
	std::cout << "Enter the number of pairs to find: ";
#endif
	int size = 0;
	std::cin >> size;
	
	index_type first;
	index_type second;
	
	for(int i = 1; i <= size; ++i) {
#ifdef TEST
		std::cout << "Enter to find a pair " << i << " (element a, element b):  ";
#endif
		std::cin >> first;
		std::cin >> second;
		data.push_back(task_pair(first, second));
	}
}

length_type search_distances_on_the_graph(const graph& data, const task_pair& pair)
{
	if(data.empty())
		return -1;
	
	index_type min_element = std::min(pair.first, pair.second);
	index_type max_element = std::max(pair.first, pair.second);	
	
	length_type length = 0;
	
	way my_way;
	my_way.insert(way_element(max_element, length));
	
	graph::const_iterator it = data.find(max_element);
	if(it == data.end())
		return -1;
	
	const node* element = &it->second;
	
	while(!element->is_head()) {
		length += element->length_to_the_parent;
		my_way.insert(way_element(element->parent_name, length));
		element = element->parent;
	}
	
	length = 0;
	index_type next_element = min_element;
	
	it = data.find(min_element);
	if(it == data.end())
		return -1;
	
	element = &it->second;
	
	while(my_way.insert(way_element(next_element, length)).second) {
		length += element->length_to_the_parent;
		next_element = element->parent_name;
		element = element->parent;
	}
	length_type main_len = my_way.find(next_element)->second + length;
	
	return main_len;
}

void create_example_graph(graph& data)
{
    node my_node;
    index_type graph_parent_name;
    
    // node 1
    index_type graph_node_name = 1;
    data.insert(graph_element(graph_node_name, my_node));
    
    // node 2
    graph_node_name = 2;
    graph_parent_name = 1;
    my_node.length_to_the_parent = 7;
    my_node.parent = &data.find(graph_parent_name)->second;
    my_node.parent_name = graph_parent_name;
    data.insert(graph_element(graph_node_name, my_node));
    
    // node 3
    graph_node_name = 3;
    graph_parent_name = 2;
    my_node.length_to_the_parent = 3;
    my_node.parent = &data.find(graph_parent_name)->second;
    my_node.parent_name = graph_parent_name;
    data.insert(graph_element(graph_node_name, my_node));
    
    // node 4
    graph_node_name = 4;
    graph_parent_name = 2;
    my_node.length_to_the_parent = 6;
    my_node.parent = &data.find(graph_parent_name)->second;
    my_node.parent_name = graph_parent_name;
    data.insert(graph_element(graph_node_name, my_node));
    
    // node 5
    graph_node_name = 5;
    graph_parent_name = 4;
    my_node.length_to_the_parent = 3;
    my_node.parent = &data.find(graph_parent_name)->second;
    my_node.parent_name = graph_parent_name;
    data.insert(graph_element(graph_node_name, my_node));
    
    // node 6
    graph_node_name = 6;
    graph_parent_name = 5;
    my_node.length_to_the_parent = 1;
    my_node.parent = &data.find(graph_parent_name)->second;
    my_node.parent_name = graph_parent_name;
    data.insert(graph_element(graph_node_name, my_node));
}

void create_example_task(task& data)
{
    data.push_back(task_pair(1, 6));
    data.push_back(task_pair(1, 3));
    data.push_back(task_pair(6, 3));
    data.push_back(task_pair(2, 5));
    data.push_back(task_pair(4, 3));
    data.push_back(task_pair(1, 2));
    data.push_back(task_pair(1, 3));
}
