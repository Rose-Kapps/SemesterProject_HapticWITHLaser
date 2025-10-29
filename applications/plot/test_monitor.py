import numpy as np
import csv
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from scipy.signal import find_peaks
import pandas as pd

# File path for CSV data
file_path = "output.csv"

# Moving average window size and peak detection threshold
window_size = 5
threshold = 0.8

# Initialize Dash app
app = dash.Dash(__name__)

# Layout of the dashboard
app.layout = html.Div([
    html.H1("Real-Time Data Plot with Peak Detection"),
    
    dcc.Graph(id='live-graph'),
    
    dcc.Interval(
        id='graph-update',
        interval=100,  # Update every 100ms
        n_intervals=0
    )
])

def read_new_data():
    """ Reads data from CSV file and returns x, y values. """
    try:
        df = pd.read_csv(file_path)
        return df.iloc[:, 0].values, df.iloc[:, 1].values  # First column = X, Second = Y
    except Exception:
        return [], []

def smooth_data(y_data, window_size=5):
    """ Applies a moving average filter. """
    if len(y_data) < window_size:
        return np.array(y_data)  # Return raw data if not enough points
    return np.convolve(y_data, np.ones(window_size) / window_size, mode='valid')

def detect_peaks(y_data, threshold=0.8):
    """ Detects peaks using scipy's find_peaks method. """
    peaks, _ = find_peaks(y_data, height=threshold)
    return peaks

@app.callback(
    Output('live-graph', 'figure'),
    Input('graph-update', 'n_intervals')
)
def update_graph(n):
    # Read new data
    new_x, new_y = read_new_data()
    if len(new_x) < 2 or len(new_y) < 2:
        return go.Figure()  # Return an empty figure if no data
    
    # Smooth the data
    smoothed_y = smooth_data(new_y, window_size)
    smoothed_x = new_x[:len(smoothed_y)]  # Ensure same length
    
    # Detect peaks
    peaks = detect_peaks(smoothed_y, threshold)
    peaks_x = [smoothed_x[i] for i in peaks]
    peaks_y = [smoothed_y[i] for i in peaks]
    
    # Determine inside/outside state
    state = "outside"
    if len(peaks) >= 2:
        state = "inside" if state == "outside" else "outside"
    
    # Create figure
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=new_x, y=new_y, mode='lines+markers', name='Raw Data'))
    #fig.add_trace(go.Scatter(x=smoothed_x, y=smoothed_y, mode='lines', name='Smoothed Data', line=dict(width=2)))
    fig.add_trace(go.Scatter(x=peaks_x, y=peaks_y, mode='markers', name='Peaks', marker=dict(color='red', size=8)))
    fig.update_layout(title=f"Real-Time Data Plot (State: {state})")

    return fig

# Run the Dash app
if __name__ == '__main__':
    app.run_server(debug=True)
