import streamlit as st
import re
import numpy as np
import os
from bs4 import BeautifulSoup
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def extract_coordinates_from_html(html_content):
    soup = BeautifulSoup(html_content, "html.parser")
    x_coords, y_coords = [], []
    script_texts = soup.find_all("script")
    for script in script_texts:
        move_to_matches = re.findall(r'ctx\.moveTo\(([^)]+)\)', script.text)
        line_to_matches = re.findall(r'ctx\.lineTo\(([^)]+)\)', script.text)
        all_matches = move_to_matches + line_to_matches
        for match in all_matches:
            coords = match.split(",")
            x_coords.append(float(coords[0].strip()))
            y_coords.append(float(coords[1].strip()))
    return x_coords, y_coords

def solve_tsp_google_or(coords):
    manager = pywrapcp.RoutingIndexManager(len(coords), 1, 0)
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(np.linalg.norm(np.array(coords[from_node]) - np.array(coords[to_node])))
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        return route
    return None

def save_to_txt(filename, data):
    with open(filename, "w") as f:
        f.write("\n".join(map(str, data)))

def main():
    st.title("Vector Coordinate Extractor & Optimizer")
    uploaded_file = st.file_uploader("Upload an HTML file", type=["html"])
    if uploaded_file is not None:
        html_content = uploaded_file.read().decode("utf-8")
        x_coords, y_coords = extract_coordinates_from_html(html_content)
        coordinates = list(zip(x_coords, y_coords))
        optimal_route = solve_tsp_google_or(coordinates)
        if optimal_route:
            optimized_x = [x_coords[i] for i in optimal_route]
            optimized_y = [y_coords[i] for i in optimal_route]
            st.download_button("Download Optimized X Coordinates", "\n".join(map(str, optimized_x)), "optimized_x.txt")
            st.download_button("Download Optimized Y Coordinates", "\n".join(map(str, optimized_y)), "optimized_y.txt")
            st.success("TSP optimization complete! Download your files.")
        else:
            st.error("No solution found for TSP.")

if __name__ == "__main__":
    main()
