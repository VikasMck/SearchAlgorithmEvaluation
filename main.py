from menu import print_main_menu, print_tree_graph_menu
from display import display_maze, graph_results


def main():
    while True:
        search_algorithm = print_main_menu()

        if search_algorithm == '5':
            print("Exiting program.")
            break

        if search_algorithm == '6':
            graph_results()

        search_type = print_tree_graph_menu()

        if search_type == '3':
            continue

        display_maze(search_algorithm, search_type)


if __name__ == '__main__':
    main()
