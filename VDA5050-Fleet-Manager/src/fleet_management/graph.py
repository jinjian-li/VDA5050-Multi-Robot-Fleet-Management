# Import necessary libraries and modules.


class Graph:
    """
    Class representing the graph, based on which the agents are controlled by the fleet manager.
    """
    # TODO: Model the layout as a graph based on the data from the LIF file.

    def __init__(self, lif_data):
        """
        Initialize the graph based on the data from the LIF file.

        :param lif_data: Data from the LIF file.
        """
        self.nodes = self.get_nodes(lif_data)
        self.edges = self.get_edges(lif_data)
        self.stations = self.get_stations(lif_data)
        self.dwelling_nodes = self.get_dwelling_nodes(lif_data)

    def get_nodes(self, lif_data):
        """
        Get the nodes from the LIF data.

        :param lif_data: Data from the LIF file.
        :return: The nodes in the graph.
        """
        nodes = {}
        if 'layouts' in lif_data and len(lif_data['layouts']) > 0:
            layout = lif_data['layouts'][0]
            for node in layout.get('nodes', []):
                node_id = node.get('nodeId')
                if node_id:
                    nodes[node_id] = {
                        'nodeId': node_id,
                        'mapId': node.get('mapId'),
                        'position': node.get('nodePosition', {}),
                        'vehicleType': node.get('vehicleTypeNodeProperties', [{}])[0].get('vehicleTypeId') if node.get('vehicleTypeNodeProperties') else None
                    }
        return nodes

    def get_edges(self, lif_data):
        """
        Get the edges from the LIF data.

        :param lif_data: Data from the LIF file.
        :return: The edges in the graph.
        """
        edges = {}
        if 'layouts' in lif_data and len(lif_data['layouts']) > 0:
            layout = lif_data['layouts'][0]
            for edge in layout.get('edges', []):
                edge_id = edge.get('edgeId')
                if edge_id:
                    # Get edge properties
                    vehicle_props = edge.get('vehicleTypeEdgeProperties', [{}])[0] if edge.get('vehicleTypeEdgeProperties') else {}
                    edges[edge_id] = {
                        'edgeId': edge_id,
                        'startNodeId': edge.get('startNodeId'),
                        'endNodeId': edge.get('endNodeId'),
                        'rotationAllowed': vehicle_props.get('rotationAllowed', True)
                    }
        return edges

    def get_stations(self, lif_data):
        """
        Get the stations from the LIF data.
        At station nodes the agents can interact with the environment.

        :param lif_data: Data from the LIF file.
        :return: The stations in the graph.
        """
        stations = {}
        if 'layouts' in lif_data and len(lif_data['layouts']) > 0:
            layout = lif_data['layouts'][0]
            for station in layout.get('stations', []):
                station_id = station.get('stationId')
                if station_id:
                    stations[station_id] = {
                        'stationId': station_id,
                        'stationType': station.get('stationType'),
                        'position': station.get('stationPosition', {}),
                        'interactionNodeIds': station.get('interactionNodeIds', [])
                    }
        return stations

    def get_dwelling_nodes(self, lif_data) -> list:
        """
        Get the dwelling nodes.
        Dwelling nodes are nodes, where the agents can park and wait for the next task.
        Number of dwelling nodes must be equal or greater than the number of agents.

        :param lif_data: The data from the LIF file.
        :return: A list of node IDs of the dwelling nodes.
        """
        dwelling_nodes = []
        if 'layouts' in lif_data and len(lif_data['layouts']) > 0:
            layout = lif_data['layouts'][0]
            # Dwelling nodes are typically charging stations
            for station in layout.get('stations', []):
                if station.get('stationType') == 'CHARGING':
                    # Get the interaction nodes for this charging station
                    interaction_nodes = station.get('interactionNodeIds', [])
                    dwelling_nodes.extend(interaction_nodes)
        return dwelling_nodes
