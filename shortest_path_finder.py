
import networkx as nx
import folium
import tkinter as tk
from tkinter import messagebox
import webbrowser
import heapq

# Create the graph with nodes and edges
def create_graph():
    G = nx.Graph()
    # Define the locations (cities) and their geographical coordinates (latitude, longitude)
    locations = {
        "Sohag": (26.558, 31.695),
        "Assiut": (27.1789, 31.1853),
        "Minya": (28.1022, 30.7517),
        "Beni Suef": (29.0731, 31.0992),
        "Giza": (30.008, 31.210),
        "Cairo": (30.0444, 31.2357),
        "Qena": (26.155, 32.711),
        "Luxor": (25.6872, 32.6396),
        "Aswan": (24.0889, 32.8998)
    }
    # Define the distances between cities (edges with weights)
    distances = {
        ("Sohag", "Assiut"): 100,
        ("Assiut", "Minya"): 150,
        ("Minya", "Beni Suef"): 130,
        ("Beni Suef", "Giza"): 110,
        ("Giza", "Cairo"): 30,
        ("Sohag", "Qena"): 50,
        ("Qena", "Luxor"): 70,
        ("Luxor", "Aswan"): 150
    }
    # Add nodes (cities) to the graph with their coordinates
    for loc, coord in locations.items():
        G.add_node(loc, pos=coord)
    # Add edges (distances between cities) to the graph
    for (loc1, loc2), dist in distances.items():
        G.add_edge(loc1, loc2, weight=dist)
    return G

# A* algorithm to find the shortest path
def a_star(graph, start, goal):
    # Heuristic function to estimate the distance between two points 
    def heuristic(pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

    # Priority queue to explore the graph (cost, path)
    queue = [(0, [start])]
    visited = set()  # Set to keep track of visited nodes
    while queue:
        cost, path = heapq.heappop(queue)  # Pop the node with the lowest cost
        node = path[-1]
        # If the destination node is reached, return the path and total cost
        if node == goal:
            return path, cost
        if node not in visited:
            visited.add(node)
            # Explore neighbors (adjacent cities) and calculate the cost to each
            for neighbor in graph.neighbors(node):
                weight = graph[node][neighbor]['weight']
                h = heuristic(graph.nodes[neighbor]['pos'], graph.nodes[goal]['pos'])  # Heuristic estimate
                # Push the next node to the priority queue with the updated cost
                heapq.heappush(queue, (cost + weight + h, path + [neighbor]))
    return None, None  # Return None if no path is found

# Create an interactive map showing the path
def create_map(path, graph):
    # Create a Folium map centered on the first city in the path
    m = folium.Map(location=graph.nodes[path[0]]["pos"], zoom_start=6)
    coordinates = [graph.nodes[node]["pos"] for node in path]
    folium.PolyLine(coordinates, color="blue", weight=3).add_to(m)
    for node in path:
        folium.Marker(graph.nodes[node]["pos"], tooltip=node).add_to(m)
    m.save("shortest_path.html")
    webbrowser.open("shortest_path.html")

def gui():
    def find_path():
        destination = entry.get()  # Get the destination city from the user input
        if destination not in G.nodes:  # Check if the destination is valid
            messagebox.showerror("Error", "Invalid destination!")  # Show an error message if invalid
            return
        # Use A* algorithm to find the shortest path from "Sohag" to the destination
        path, distance = a_star(G, "Sohag", destination)
        if path: 
            create_map(path, G) 
            messagebox.showinfo("Result", f"Path: {' -> '.join(path)}\nDistance: {round(distance, 2)} km")
        else:
            messagebox.showerror("Error", "No path found!")  

    # Create the main window for the GUI
    root = tk.Tk()
    root.title("Shortest Path Finder")  
    tk.Label(root, text="Enter Destination (e.g., Cairo):").pack(pady=5)  # Add a label
    entry = tk.Entry(root)  # Create an input field for the destination
    entry.pack(pady=5)
    tk.Button(root, text="Find Path", command=find_path).pack(pady=10)  # Add a button to find the path
    root.mainloop()  # Start the Tkinter event loop to run the GUI

# Run the program
G = create_graph()  # Create the graph with cities and distances
gui()  # Start the GUI to interact with the user
