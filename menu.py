# Basic menu, can be done fancier later
def print_main_menu():
    print("Choose a search algorithm:")
    print("1. BFS")
    print("2. DFS")
    print("3. UCS")
    print("4. A*")
    print("5. Run Analysis (Runs 1440 Mazes)")
    print("6. Exit")

    while True:
        user_input = input("Enter your user_input (1-6): ").strip()
        if user_input in ('1', '2', '3', '4', '5', '6'):
            return user_input
        print("Please enter a number between 1 and 5.")


def print_tree_graph_menu():
    print("Choose a search type")
    print("1. Tree Search")
    print("2. Graph Search")
    print("3. Back to main menu")

    while True:
        user_input = input("Enter your user_input (1-3): ").strip()
        if user_input in ('1', '2', '3'):
            return user_input
        print("Please enter a number between 1 and 3.")
