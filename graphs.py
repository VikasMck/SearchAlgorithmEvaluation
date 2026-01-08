import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# for grids
sns.set_style("whitegrid")


# due to tree being much larger in terms of data, most graphs are separated


# take average time to solve a maze and plot
def algorithm_average_time_plot(results_data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    fig.suptitle('Average Time per Algorithm (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]
        data_average = data.groupby('Search_Algorithm_Name')['Time'].mean().sort_values()
        data_average.plot(kind='bar', ax=axes[column], color='blue' if search_type == 'graph' else 'green')
        axes[column].set_title(f'{search_type.upper()}')
        axes[column].set_xlabel('Average Time (seconds)')
        axes[column].tick_params(axis='x', rotation=45, labelsize=10)

    plt.tight_layout()
    plt.show()


# which algorithm is affected by size the most
def maze_size_affect_plot(results_data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    fig.suptitle('Algorithm Scalability (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]
        data_filter = data.groupby(['Maze_Size', 'Search_Algorithm_Name'])['Time'].mean().unstack()
        data_filter.plot(kind='line', marker='o', ax=axes[column])
        axes[column].set_title(f'{search_type.upper()}')
        axes[column].set_xlabel('Maze Size')
        axes[column].set_ylabel('Time (seconds)')
        axes[column].legend(fontsize=10)
        axes[column].tick_params(axis='x', rotation=45, labelsize=10)

    plt.tight_layout()
    plt.show()


# similar to size, just by density
def maze_density_affect_plot(results_data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    fig.suptitle('Density Scalability (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]
        data_filter = data.groupby(['Maze_Density', 'Search_Algorithm_Name'])['Time'].mean().unstack()
        data_filter.plot(kind='line', marker='o', ax=axes[column])
        axes[column].set_title(f'{search_type.upper()}')
        axes[column].set_xlabel('Maze Density')
        axes[column].set_ylabel('Time (seconds)')
        axes[column].legend(fontsize=10)
        axes[column].tick_params(axis='x', rotation=45, labelsize=10)

    plt.tight_layout()
    plt.show()


# better understanding with the help of bar plot for both density and size
def algorithms_per_density_size_plot(results_data):
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Algorithm by Size and Density (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]

        size = data.groupby(['Maze_Size', 'Search_Algorithm_Name'])['Time'].mean().unstack()
        size.plot(kind='bar', ax=axes[column, 0])
        axes[column, 0].set_title(f'{search_type.upper()}: Size')
        axes[column, 0].set_xlabel('Maze Size')
        axes[column, 0].set_ylabel('Time (seconds)')
        axes[column, 0].legend(fontsize=10)
        axes[column, 0].tick_params(axis='x', rotation=45, labelsize=10)

        obstacle_density = data.groupby(['Maze_Density', 'Search_Algorithm_Name'])['Time'].mean().unstack()
        obstacle_density.plot(kind='bar', ax=axes[column, 1])
        axes[column, 1].set_title(f'{search_type.upper()}: Density')
        axes[column, 1].set_xlabel('Maze Density')
        axes[column, 1].set_ylabel('Time (seconds)')
        axes[column, 1].legend(fontsize=10)
        axes[column, 1].tick_params(axis='x', rotation=45, labelsize=10)

    plt.tight_layout()
    plt.show()


# this one is very hard to present, so more for us to see. Expand to see better, long graph
def start_to_goal_performance_plot(results_data):
    regions_text = {
        1: 'BottomLeft', 2: 'TopLeft', 3: 'BottomRight', 4: 'TopRight', 5: 'Centre',
        '1': 'BottomLeft', '2': 'TopLeft', '3': 'BottomRight', '4': 'TopRight', '5': 'Centre'
    }

    # creating A -> B column by mapping regions text to 1-5 and concat
    results_df_copy = results_data.copy()
    results_df_copy['Start_Region'] = results_df_copy['Start_Region'].map(regions_text).fillna(
        results_df_copy['Start_Region'])
    results_df_copy['Goal_Region'] = results_df_copy['Goal_Region'].map(regions_text).fillna(
        results_df_copy['Goal_Region'])
    results_df_copy['Route'] = results_df_copy['Start_Region'] + ' -> ' + results_df_copy['Goal_Region']

    fig, axes = plt.subplots(1, 2, figsize=(25, 8))
    fig.suptitle('Performance Per Route (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_df_copy[results_df_copy['Search_Type_Name'] == search_type]
        data_filter = data.groupby(['Route', 'Search_Algorithm_Name'])['Time'].mean().unstack()
        data_filter.plot(kind='bar', ax=axes[column])
        axes[column].set_title(f'{search_type.upper()}')
        axes[column].set_xlabel('Start Region -> Goal Region')
        axes[column].set_ylabel('Average Time (seconds)')
        axes[column].legend(fontsize=10)
        axes[column].tick_params(axis='x', rotation=45, labelsize=10)

    plt.tight_layout()
    plt.show()


# sometimes more nodes = faster due to how they are handled, so wanted to see
def node_analysis_plot(results_data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 8))
    fig.suptitle('Nodes Expanded per Time (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]
        for algorithm in data['Search_Algorithm_Name'].unique():
            algorithm_data = data[data['Search_Algorithm_Name'] == algorithm]
            axes[column].scatter(algorithm_data['Num_Nodes_Expanded'], algorithm_data['Time'], label=algorithm,
                                 alpha=0.6, s=20)
        axes[column].set_title(search_type.upper())
        axes[column].set_xlabel('Nodes Expanded')
        axes[column].set_ylabel('Time (seconds)')
        axes[column].legend(fontsize=10)

    plt.tight_layout()
    plt.show()


def algorithm_peak_storage_plot(results_data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 10))
    fig.suptitle('Peak Memory Usage Per Algorithm (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]

        means = data.groupby('Search_Algorithm_Name')['Peak_Memory_Usage'].mean()

        sns.barplot(x='Search_Algorithm_Name', y='Peak_Memory_Usage', data=data, ax=axes[column], estimator='mean',
                    color='blue' if search_type == 'graph' else 'green')
        axes[column].set_title(f'{search_type.upper()}: Size')

        axes[column].set_xlabel('Search Algorithm')
        axes[column].set_ylabel('Peak Memory Usage (Bytes)')
        axes[column].tick_params(axis='x', rotation=45, labelsize=10)
        axes[column].ticklabel_format(style='plain', axis='y')
        plt.yticks(sorted(means.values))

    plt.tight_layout()
    plt.show()


def path_expanded_memory_usage_plot(results_data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 10))
    fig.suptitle('Path Expanded Memory Usage Per Algorithm (Graph v Tree)', fontsize=15, fontweight='bold')

    for column, search_type in enumerate(['graph', 'tree']):
        data = results_data[results_data['Search_Type_Name'] == search_type]

        data['Peak_Memory_Usage_KB'] = data['Peak_Memory_Usage'] / 1024
        sns.scatterplot(x='Num_Nodes_Expanded', y='Peak_Memory_Usage_KB', data=data, hue='Algorithm_And_Search_Name',
                        ax=axes[column])
        axes[column].set_xlabel('Num Nodes Expanded')
        axes[column].set_title(f'{search_type.upper()}')
        axes[column].set_ylabel('Peak Memory Usage (kB)')
        axes[column].ticklabel_format(style='plain', axis='y')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    results_df = pd.read_csv('results_df.csv')
    graphs_functions = [algorithm_average_time_plot, maze_size_affect_plot, maze_density_affect_plot,
                        algorithms_per_density_size_plot, start_to_goal_performance_plot, node_analysis_plot,
                        algorithm_peak_storage_plot, path_expanded_memory_usage_plot]
    graphs_names = ['Average Time Per Algorithm (Line)', 'How Size Effects (Line)', 'How Density Effects (Line)',
                    'Density and Size (Bar)', 'Start to Goal Combinations (Bar)', 'Nodes per Time (Scatter)',
                    'Peak Memory Usage (Bar)', 'Path Expanded To Memory Usage (Scatter)']

    while True:
        for i, name in enumerate(graphs_names, 1):
            print(f"{i}. {name}")
        print("9. Exit")

        choice = input("\nPick 1-9: ").strip()

        if choice == '9':
            break
        elif choice.isdigit() and 1 <= int(choice) <= 8:
            graphs_functions[int(choice) - 1](results_df)
        else:
            print("Wrong input")
