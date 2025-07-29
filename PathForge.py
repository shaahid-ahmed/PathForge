import ezdxf
from ezdxf import transform
import math
import numpy as np
from rdp import rdp
from shapely.geometry import Polygon
from shapely.geometry import LineString
from scipy.interpolate import BSpline, UnivariateSpline
from networkx import Graph, connected_components
class DXF_to_GCODE():
    def __init__(self,input_dxf_file, output_gcode_file, feed_rate, dwell_time, accuracy, saccuracy,kerf,e,layers):
        self.layers = layers
        self.dxf_to_gcode(input_dxf_file, output_gcode_file, feed_rate, dwell_time, accuracy, saccuracy,kerf,e)
    def get_entity_start_point(self, entity):
        if entity.dxftype() == 'LINE':
            return (round(entity.dxf.start[0], 2), round(entity.dxf.start[1], 2))
        elif entity.dxftype() == 'ARC':
            return (round(entity.start_point[0], 2), round(entity.start_point[1], 2))
        elif entity.dxftype() == 'SPLINE':
            control_point = entity.control_points[0][:2]
            return (round(control_point[0], 2), round(control_point[1], 2))
        elif entity.dxftype() == 'CIRCLE':
            return (round(entity.dxf.center[0] - entity.dxf.radius, 2), round(entity.dxf.center[1], 2))
        else:
            return None

    def get_entity_end_point(self, entity):
        if entity.dxftype() == 'LINE':
            return (round(entity.dxf.end[0], 2), round(entity.dxf.end[1], 2))
        elif entity.dxftype() == 'ARC':
            return (round(entity.end_point[0], 2), round(entity.end_point[1], 2))
        elif entity.dxftype() == 'SPLINE':
            control_point = entity.control_points[-1][:2]
            return (round(control_point[0], 2), round(control_point[1], 2))
        elif entity.dxftype() == 'CIRCLE':
            return (round(entity.dxf.center[0] + entity.dxf.radius, 2), round(entity.dxf.center[1], 2))
        else:
            return None
    def find_components_in_entities(self, entities):
        adjacency_list = {}
        for (e, start, end) in entities:
            if start not in adjacency_list:
                adjacency_list[start] = []
            if end not in adjacency_list:
                adjacency_list[end] = []
            adjacency_list[start].append(end)
            adjacency_list[end].append(start)

        G = Graph(adjacency_list)
        components = list(connected_components(G))

        entities_map = {}
        for component in components:
            component_key = frozenset(component)
            entities_map[component_key] = [e for e, start, end in entities if start in component or end in component]

        return entities_map
    def find_leftmost(self, lines):
        leftmost = lines[0]
        for line in lines:
            if line[0][0] < leftmost[0][0] or (line[0][0] == leftmost[0][0] and line[0][1] < leftmost[0][1]):
                leftmost = line
        return leftmost
    def is_point_inside_line(self, point, line):
        return min(line[0][0], line[1][0]) <= point[0] <= max(line[0][0], line[1][0]) and \
            min(line[0][1], line[1][1]) <= point[1] <= max(line[0][1], line[1][1])
    def reorder(self, arr):
        # Convert tuples to lists to allow mutable operations
        arr = [[list(pair[0]), list(pair[1])] for pair in arr]

        # Initialize the reordered list with the first element
        reordered = [arr[0]]

        # Remove the first element from the original list
        arr.pop(0)

        # Repeatedly find and reorder elements until the reordered list is complete
        while arr:
            # Get the last point in the reordered list
            last_point = reordered[-1][1]

            # Find the next edge that connects to the last point
            for i, edge in enumerate(arr):
                if edge[0] == last_point:
                    next_edge = edge
                    break
                elif edge[1] == last_point:
                    next_edge = [edge[1], edge[0]]
                    break

            # Add the next edge to the reordered list
            reordered.append(next_edge)

            # Remove the used edge from the original list
            arr.pop(i)

        # Convert lists back to tuples
        reordered = [(tuple(pair[0]), tuple(pair[1])) for pair in reordered]
        arranged = []
        line_indices = []
        for i, line in enumerate(reordered):
            arranged.append(line)
            line_indices.append(i)

        return arranged, line_indices

    def arrange_lines(self, lines):
        arranged = []
        line_indices = []  
        used_indices = set() 
        
        leftmost = self.find_leftmost(lines)
        leftmost_index = lines.index(leftmost)
        arranged.append(leftmost)
        line_indices.append(leftmost_index)
        used_indices.add(leftmost_index)
        
        next_point = leftmost[1]
        
        while len(used_indices) < len(lines):
            found = False
            for i, line in enumerate(lines):
                if i not in used_indices:
                    if line[0] == next_point:
                        arranged.append(line)
                        line_indices.append(i)
                        used_indices.add(i)
                        next_point = line[1]
                        found = True
                        break
                    elif line[1] == next_point:
                        arranged.append(line)
                        line_indices.append(i)
                        used_indices.add(i)
                        next_point = line[0]
                        found = True
                        break
                    elif self.is_point_inside_line(next_point, line):
                        arranged.append(line)
                        line_indices.append(i)
                        used_indices.add(i)
                        next_point = line[1] if line[0] == next_point else line[0]
                        found = True
                        break
            if not found:
                for i, line in enumerate(lines):
                    if i not in used_indices:
                        next_point = line[0]
                        found = True
                        break
            if not found:
                break
        
        for i, line in enumerate(lines):
            if i not in used_indices:
                arranged.append(line)
                line_indices.append(i)
        
        return arranged, line_indices
    def calculate_arc_angles(self, start_point, end_point, center_point, radius):
        start_angle = math.degrees(math.atan2(start_point[1] - center_point[1], start_point[0] - center_point[0]))
        end_angle = math.degrees(math.atan2(end_point[1] - center_point[1], end_point[0] - center_point[0]))
        return start_angle, end_angle
    def determine_arc_direction(self, center, start, end):
        # Vector from center to start
        vector_start = (start[0] - center[0], start[1] - center[1])
        
        # Vector from center to end
        vector_end = (end[0] - center[0], end[1] - center[1])
        
        # Calculate the cross product of the vectors
        cross_product = vector_start[0] * vector_end[1] - vector_start[1] * vector_end[0]
        
        # Determine the direction based on the cross product
        if cross_product > 0:
            return "counterclockwise"
        else:
            return "clockwise"
    def distance_between_left_and_right(self, coordinates):
        unique_points = set()
        for pair in coordinates:
            unique_points.update(pair)

        leftmost_point = min(unique_points, key=lambda p: p[0])
        rightmost_point = max(unique_points, key=lambda p: p[0])

        # distance = math.sqrt((rightmost_point[0] - leftmost_point[0]) ** 2 + (rightmost_point[1] - leftmost_point[1]) ** 2)
        distance = rightmost_point[0]-leftmost_point[0]
        return round(distance,2)
    def find_index(self, layers, entities):
        for index, layer in enumerate(layers):
            new_entities_list = [str(entity) for entity in entities]
            if [str(entity) for entity in layer["name"]] == new_entities_list:
                return index,layer["cut_type"]
        return -1,'not found'  # Return -1 if the entity is not found
    def offset_point(self,start, end, offset_distance,cut_type):
        """
        Calculate a point offset_distance away from the start point in the direction of the end point.
        """
        line = LineString([start, end])
        length = line.length
        offset_ratio = offset_distance / length
        dx = (end[0] - start[0]) * offset_ratio
        dy = (end[1] - start[1]) * offset_ratio
        if cut_type=='outer':
            return start[0] - dx, start[1] - dy
        else:
            return start[0] + dx, start[1] + dy
    def find_midpoint(self,coords):
        """
        Find the midpoint of a list of coordinates.

        :param coords: List of tuples, where each tuple represents a coordinate (x, y)
        :return: Tuple representing the midpoint (x, y)
        """
        if not coords:
            raise ValueError("The list of coordinates is empty")
        coords=list(set([coord for sublist in coords for coord in sublist]))
        n = len(coords)
        sum_x = sum(coord[0] for coord in coords)
        sum_y = sum(coord[1] for coord in coords)

        midpoint_x = sum_x / n
        midpoint_y = sum_y / n

        return midpoint_x, midpoint_y
    def find_tangent_of_spline(self, x_list, y_list, point):
        """
        Finds the tangent of a spline curve at a given point.
        
        Args:
            x_list (list): A list of x-coordinates for the data points.
            y_list (list): A list of y-coordinates for the data points.
            point (float): The point where the tangent is to be calculated.
            
        Returns:
            tuple: A tuple containing the point (x, y) and the tangent slope at that point.
        """
        sorted_indices = np.argsort(x_list)
        x = np.array(x_list)[sorted_indices]
        y = np.array(y_list)[sorted_indices]

        # Ensure x values are strictly increasing
        unique_x, unique_indices = np.unique(x, return_index=True)
        unique_y = y[unique_indices]

        # Check if there are at least 4 unique points (for cubic spline, k=3)
        if len(unique_x) < 4:
            # Treat it as a line and find the slope
            if len(unique_x) == 1:
                return float('inf'),point
            # Compute the slope of the line using the first two unique points
            slope = (unique_y[1] - unique_y[0]) / (unique_x[1] - unique_x[0])
            x0 = point
            y0 = unique_y[0] + slope * (x0 - unique_x[0])
            return slope, (x0, y0)

        # Create a spline object
        spl = UnivariateSpline(unique_x, unique_y, s=0)

        # Evaluate the spline and its derivative at the given point
        x0 = point
        y0 = spl(x0)
        dy0 = spl.derivative(n=1)(x0)
        return dy0, (x0, y0)
    def find_tangent_of_arc(self,start_point, end_point, center, radius, point):
        """
        Finds the tangent of an arc at a given point.
        
        Args:
            start_point (tuple): The starting point of the arc (x, y).
            end_point (tuple): The ending point of the arc (x, y).
            center (tuple): The center of the arc (x, y).
            radius (float): The radius of the arc.
            point (tuple): The point where the tangent is to be calculated (x, y).
            
        Returns:
            tuple: A tuple containing the point (x, y) on the arc and the tangent slope at that point.
        """
        # Calculate the angle subtended by the arc
        start_angle = np.arctan2(start_point[1] - center[1], start_point[0] - center[0])
        end_angle = np.arctan2(end_point[1] - center[1], end_point[0] - center[0])
        angle = np.arctan2(point[1] - center[1], point[0] - center[0])
        
        # Adjust end_angle if it's less than start_angle
        if end_angle < start_angle:
            end_angle += 2 * np.pi
        
        # Adjust angle if it's outside the range [start_angle, end_angle]
        while angle < start_angle:
            angle += 2 * np.pi
        while angle > end_angle:
            angle -= 2 * np.pi
        
        # Determine the nearest point on the arc to the given point
        nearest_angle = start_angle if abs(angle - start_angle) < abs(angle - end_angle) else end_angle
        nearest_point = (center[0] + radius * np.cos(nearest_angle), center[1] + radius * np.sin(nearest_angle))
        
        # Calculate the tangent slope at the nearest point
        if abs(nearest_angle - start_angle) < abs(nearest_angle - end_angle):
            tangent_slope = -1 / np.tan(nearest_angle)
        else:
            tangent_slope = -1 / np.tan(nearest_angle + np.pi)
        
        return tangent_slope, nearest_point
    def slope(self,entity,start,end,point,saccuracy):
        if entity.dxftype() == 'LINE':
            x1, y1 = start[0],start[1]
            x2, y2 = end[0],end[1]
            
            # Calculate change in Y and change in X
            delta_y = y2 - y1
            delta_x = x2 - x1
            
            # Check for division by zero
            if delta_x == 0:
                return float('inf'),point  # Vertical line, return positive infinity
            
            # Calculate the slope
            slope = delta_y / delta_x
            
            return slope, point
        elif entity.dxftype() == 'ARC':
            center = entity.dxf.center
            radius = entity.dxf.radius
            start_point = start
            end_point = end
            a,b=self.find_tangent_of_arc(start_point, end_point, center, radius, point)
            return a,b
        elif entity.dxftype() == 'SPLINE':
            control_points = entity.control_points
            control_points = np.array([i[:-1] for i in control_points])
            degree = entity.dxf.degree
            knots = entity.knots
            np.sort(knots)
            spline = BSpline(knots, control_points, degree)
            t = np.linspace(knots[degree], knots[-degree], 10*saccuracy)
            spline_points = spline(t)
            simplified_points = rdp(spline_points, epsilon=0.01)
            x, y = simplified_points[:, 0], simplified_points[:, 1] 
            a,b=self.find_tangent_of_spline(x, y, point[0])
            return a,b
        
    def find_point_on_angle_bisector(self,slope1, slope2, intersection_point, d):
        """
        Find the point at a given distance 'd' from the point of intersection on the angle bisector of two lines.
        
        Args:
            slope1 (float): Slope of the first line.
            slope2 (float): Slope of the second line.
            intersection_point (tuple): Point of intersection of the two lines (x, y).
            d (float): Distance from the point of intersection along the angle bisector.
            
        Returns:
            tuple: Point at the given distance 'd' along the angle bisector (x, y).
        """
        # Calculate the angle bisector slope
        angle1 = math.atan(slope1)
        angle2 = math.atan(slope2)
        bisector_angle = (angle1 + angle2) / 2
        bisector_slope = math.tan(bisector_angle)
        
        # Find the equation of the angle bisector line
        x0, y0 = intersection_point
        bisector_line_equation = lambda x: y0 + bisector_slope * (x - x0)
        
        # Calculate the point at the given distance 'd' along the angle bisector
        angle_from_x_axis = math.atan(bisector_slope)
        dx = d * math.cos(angle_from_x_axis)
        dy = d * math.sin(angle_from_x_axis)
        
        x = x0 + dx
        y = bisector_line_equation(x)
        
        return x, y
    def distance_point_to_line_segment(self,point, segment_start, segment_end):
        """
        Calculate the distance between a point and a line segment.
        
        Args:
            point (tuple): The coordinates of the point (x, y).
            segment_start (tuple): The coordinates of the start point of the line segment (x, y).
            segment_end (tuple): The coordinates of the end point of the line segment (x, y).
            
        Returns:
            float: The minimum distance between the point and the line segment.
        """
        x, y = point
        x1, y1 = segment_start
        x2, y2 = segment_end
        
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == dy == 0:  # The segment is a point
            return np.sqrt((x - x1) ** 2 + (y - y1) ** 2)

        t = ((x - x1) * dx + (y - y1) * dy) / (dx ** 2 + dy ** 2)
        t = max(0, min(1, t))  # Clamp t to the interval [0, 1]

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        distance = np.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
        return distance
    def ray_intersects_edge(self,ray_start, ray_end, vertex1, vertex2):
        """
        Determine if a ray intersects with an edge.
        
        Args:
            ray_start (tuple): The start point of the ray.
            ray_end (tuple): The end point of the ray.
            vertex1 (tuple): The first vertex of the edge.
            vertex2 (tuple): The second vertex of the edge.
            
        Returns:
            bool: True if the ray intersects with the edge, False otherwise.
        """
        x1, y1 = ray_start
        x2, y2 = ray_end
        x3, y3 = vertex1
        x4, y4 = vertex2
        
        # Calculate the direction vectors
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x4 - x3
        dy2 = y4 - y3
        
        # Calculate the determinant
        det = dx1 * dy2 - dy1 * dx2
        
        # Check if the lines are parallel
        if det == 0:
            return False
        
        # Calculate the parameter t for the intersection point
        t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / det
        
        # Check if the intersection point is on the ray
        if t >= 0 and t <= 1:
            # Calculate the parameter u for the intersection point
            u = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / det
            
            # Check if the intersection point is on the edge
            if u >= 0 and u <= 1:
                return True
        
        return False
    def is_point_inside_polygon(self,point, polygon):
        """
        Determine if a point lies inside a polygon and calculate the minimum distance to the polygon.

        Args:
            point (tuple): The point to check, represented as (x, y) coordinates.
            polygon (list): A list of tuples, where each tuple represents a vertex of the polygon.

        Returns:
            float: The minimum distance between the point and the polygon. Returns 0 if the point is inside the polygon.
        """
        x, y = point
        
        # Create a ray casting from the point in the positive x-direction
        ray_start = (x, y - 1e-9)  # Slightly adjust the ray start point to avoid corner cases
        ray_end = (x + 1e9, y)  # Extend the ray infinitely in the positive x-direction
        
        # Initialize the crossing number
        crossing_number = 0
        
        # Initialize the minimum distance
        min_distance = float('inf')
        
        # Iterate over the polygon edges
        for i in range(len(polygon)):
            vertex1 = polygon[i]
            vertex2 = polygon[(i + 1) % len(polygon)]
            
            # Check if the ray intersects with the edge
            if self.ray_intersects_edge(ray_start, ray_end, vertex1, vertex2):
                crossing_number += 1
            
            # Calculate the distance from the point to the edge
            distance = self.distance_point_to_line_segment(point, vertex1, vertex2)
            min_distance = min(min_distance, distance)
        
        # If the crossing number is odd, the point is inside the polygon
        if crossing_number % 2 == 1:
            return 0
        else:
            return min_distance
    def dxf_to_gcode(self, input_dxf_file, output_gcode_file, feed_rate, dwell_time, accuracy, saccuracy,kerf,e):
            with open(output_gcode_file, 'w') as gcode_file:
                gcode_file.write("G94\n")
                gcode_file.write(f"F{feed_rate}\n")
                gcode_file.write("G21\n")

                doc = ezdxf.readfile(input_dxf_file)
                msp = doc.modelspace()
                layers = []

                entities = []
                for entity in msp:
                    start = self.get_entity_start_point(entity)
                    end = self.get_entity_end_point(entity)
                    if start and end:
                        entities.append((entity, start, end))
                components_entities = self.find_components_in_entities(entities)
                layers += list(components_entities.values())
                for entities in layers:
                    entity_list =[]
                    for entity in entities:
                        start = self.get_entity_start_point(entity)
                        end = self.get_entity_end_point(entity)
                        entity_list += [(start, end)]
                    _,cut_type=self.find_index(self.layers,entities)
                    if cut_type=='outer':
                        d=self.distance_between_left_and_right(entity_list)
                        scaling_factor = (d+(kerf*2))/d
                        ezdxf.transform.scale_uniform(entities, scaling_factor)
                    else:
                        d=self.distance_between_left_and_right(entity_list)
                        scaling_factor = (d-(kerf*2))/d
                        ezdxf.transform.scale_uniform(entities, scaling_factor)

                    gcode_file.write("M05\n")
                    lines = []
                    for x in entities:
                        lines += [(self.get_entity_start_point(x), self.get_entity_end_point(x))]
                    if lines:
                        arranged_lines, line_indices = self.arrange_lines(lines)
                        sorted_entities = [entities[i] for i in line_indices]
                    else:
                        sorted_entities = entities
                    l=[]
                    for entity in sorted_entities:
                        l += [(self.get_entity_start_point(entity), self.get_entity_end_point(entity))]
                    if l:
                        arranged_lines, line_indices = self.reorder(l)
                        sorted_entities = [sorted_entities[i] for i in line_indices]
                    points = []
                    for line in arranged_lines:
                        points.extend(line)
                    offset = e
                    if sorted_entities[0].dxftype() in ['LINE','SPLINE','ARC']:
                        s1,_=self.slope(sorted_entities[0],start,end,arranged_lines[0][0],saccuracy)
                        s2,_=self.slope(sorted_entities[-1],start,end,arranged_lines[0][0],saccuracy)
                        x1,y1 = self.find_point_on_angle_bisector(s1, s2, arranged_lines[0][0], e)
                        x2,y2 = self.find_point_on_angle_bisector(s1, s2, arranged_lines[0][0], e*(-1))
                        if self.is_point_inside_polygon((x1,y1),points)<=self.is_point_inside_polygon((x2,y2),points):
                            x_i , y_i = x1,y1
                            x_o,y_o = x2,y2
                        else:
                            x_i , y_i = x2,y2
                            x_o,y_o = x1,y1
                    for entity in sorted_entities:
                        if sorted_entities.index(entity)==0:
                            
                            if entity.dxftype() == 'LINE':
                                start_point = entity.dxf.start
                                end_point = entity.dxf.end
                                if cut_type=='outer':
                                    gcode_file.write(f"G0 X{x_o:.{accuracy}f} Y{y_o:.{accuracy}f}\n")
                                    gcode_file.write("M03 S1000\n")
                                    gcode_file.write(f"G04 X{dwell_time}\n")
                                    gcode_file.write(f"G1 X{start_point.x:.{accuracy}f} Y{start_point.y:.{accuracy}f}\n")
                                    gcode_file.write(f"G1 X{end_point.x:.{accuracy}f} Y{end_point.y:.{accuracy}f}\n")
                                else:
                                    gcode_file.write(f"G0 X{x_i:.{accuracy}f} Y{y_i:.{accuracy}f}\n")
                                    gcode_file.write("M03 S1000\n")
                                    gcode_file.write(f"G04 X{dwell_time}\n")
                                    gcode_file.write(f"G1 X{start_point.x:.{accuracy}f} Y{start_point.y:.{accuracy}f}\n")
                                    gcode_file.write(f"G1 X{end_point.x:.{accuracy}f} Y{end_point.y:.{accuracy}f}\n")
                            elif entity.dxftype() == 'ARC':
                                i=sorted_entities.index(entity)
                                center = entity.dxf.center
                                radius = entity.dxf.radius
                                start = arranged_lines[i][0]
                                end = arranged_lines[i][1]
                                s_x = entity.start_point[0]
                                s_y = entity.start_point[1]
                                e_x = entity.end_point[0]
                                e_y = entity.end_point[1]
                                start_angle = entity.dxf.start_angle
                                end_angle = entity.dxf.end_angle
                                direction = self.determine_arc_direction(center, start, end)
                                start_angle = start_angle % 360
                                end_angle = end_angle % 360
                                angle_x = 360-(start_angle-end_angle)
                                r=radius
                                angle_span = end_angle - start_angle
                                if angle_span < 0:
                                    angle_span += 360
                                if angle_span > 180:
                                    r = radius*(-1)
                                start_angle_rad = math.radians(start_angle)
                                end_angle_rad = math.radians(end_angle)
                                start_x = arranged_lines[i][0][0]
                                start_y = arranged_lines[i][0][1]
                                end_x = arranged_lines[i][1][0]
                                end_y = arranged_lines[i][1][1]                            
                                if direction=='clockwise' and (start_x!=round(s_x,2) or start_y!=round(s_y,2)):
                                    if cut_type=='outer':
                                        gcode_file.write(f"G0 X{x_o:.{accuracy}f} Y{y_o:.{accuracy}f} Z0\n")
                                        gcode_file.write("M03 S1000\n")
                                        gcode_file.write(f"G04 X{dwell_time}\n")
                                        gcode_file.write(f"G1 X{start_x:.{accuracy}f} Y{start_y:.{accuracy}f} Z0\n")
                                        gcode_file.write(f"G2 X{end_x:.{accuracy}f} Y{end_y:.{accuracy}f} R{r:.{accuracy}f} F{feed_rate}\n")
                                    else:
                                        gcode_file.write(f"G0 X{x_i:.{accuracy}f} Y{y_i:.{accuracy}f} Z0\n")
                                        gcode_file.write("M03 S1000\n")
                                        gcode_file.write(f"G04 X{dwell_time}\n")
                                        gcode_file.write(f"G1 X{start_x:.{accuracy}f} Y{start_y:.{accuracy}f} Z0\n")
                                        gcode_file.write(f"G2 X{end_x:.{accuracy}f} Y{end_y:.{accuracy}f} R{r:.{accuracy}f} F{feed_rate}\n")
                                else:
                                    if cut_type=='outer':
                                        gcode_file.write(f"G0 X{x_o:.{accuracy}f} Y{y_o:.{accuracy}f} Z0\n")
                                        gcode_file.write("M03 S1000\n")
                                        gcode_file.write(f"G04 X{dwell_time}\n")
                                        gcode_file.write(f"G1 X{start_x:.{accuracy}f} Y{start_y:.{accuracy}f} Z0\n")
                                        gcode_file.write(f"G3 X{end_x:.{accuracy}f} Y{end_y:.{accuracy}f} R{r:.{accuracy}f} F{feed_rate}\n")
                                    else:
                                        gcode_file.write(f"G0 X{x_i:.{accuracy}f} Y{y_i:.{accuracy}f} Z0\n")
                                        gcode_file.write("M03 S1000\n")
                                        gcode_file.write(f"G04 X{dwell_time}\n")
                                        gcode_file.write(f"G1 X{start_x:.{accuracy}f} Y{start_y:.{accuracy}f} Z0\n")
                                        gcode_file.write(f"G3 X{end_x:.{accuracy}f} Y{end_y:.{accuracy}f} R{r:.{accuracy}f} F{feed_rate}\n")

                            elif entity.dxftype() == 'CIRCLE':
                                center = entity.dxf.center
                                radius = entity.dxf.radius
                                offset_start_point = self.offset_point((center[0] - radius, center[1]), (center[0] + radius, center[1]), offset,cut_type)
                                if cut_type=='outer':
                                    gcode_file.write(f"G0 X{offset_start_point[0]:.{accuracy}f} Y{offset_start_point[1]:.{accuracy}f}\n")
                                    gcode_file.write("M03 S1000\n")
                                    gcode_file.write(f"G04 X{dwell_time}\n")
                                    gcode_file.write(f"G1 X{center[0]-radius:.{accuracy}f} Y{center[1]:.{accuracy}f}\n")
                                    gcode_file.write(f"G2 X{center[0]-radius:.{accuracy}f} Y{center[1]:.{accuracy}f} I{radius:.{accuracy}f} J{0:.{accuracy}f} F{feed_rate}\n")
                                else:
                                    gcode_file.write(f"G0 X{center[0]:.{accuracy}f} Y{center[1]:.{accuracy}f}\n")
                                    gcode_file.write("M03 S1000\n")
                                    gcode_file.write(f"G04 X{dwell_time}\n")
                                    gcode_file.write(f"G1 X{center[0]-radius:.{accuracy}f} Y{center[1]:.{accuracy}f}\n")
                                    gcode_file.write(f"G2 X{center[0]-radius:.{accuracy}f} Y{center[1]:.{accuracy}f} I{radius:.{accuracy}f} J{0:.{accuracy}f} F{feed_rate}\n")
                            elif entity.dxftype() == 'SPLINE':
                                control_points = entity.control_points
                                control_points = np.array([i[:-1] for i in control_points])
                                degree = entity.dxf.degree
                                knots = entity.knots
                                np.sort(knots)
                                spline = BSpline(knots, control_points, degree)
                                t = np.linspace(knots[degree], knots[-degree], 10*saccuracy)
                                spline_points = spline(t)
                                simplified_points = rdp(spline_points, epsilon=0.01)
                                x, y = simplified_points[:, 0], simplified_points[:, 1]
                                for i in range(len(x) - 1):
                                    x1, y1 = x[i], y[i]
                                    x2, y2 = x[i + 1], y[i + 1]
                                    if i == 0:
                                        x_x,y_y=self.offset_point((x1,y1), (x[-1],y[-1]),0.5,cut_type)
                                        if cut_type=='outer':
                                            gcode_file.write(f"G0 X{x_o:.{accuracy}f} Y{y_o:.{accuracy}f}\n")
                                            gcode_file.write("M03 S1000\n")
                                            gcode_file.write(f"G04 X{dwell_time}\n")
                                            gcode_file.write(f"G1 X{x1:.{accuracy}f} Y{y1:.{accuracy}f}\n")
                                        else:
                                            gcode_file.write(f"G0 X{x_i:.{accuracy}f} Y{y_i:.{accuracy}f}\n")
                                            gcode_file.write("M03 S1000\n")
                                            gcode_file.write(f"G04 X{dwell_time}\n")
                                            gcode_file.write(f"G1 X{x1:.{accuracy}f} Y{y1:.{accuracy}f}\n")
                                    gcode_line = f"G1 X{x2:.{accuracy}f} Y{y2:.{accuracy}f}\n"
                                    gcode_file.write(gcode_line)
                        else:
                            if entity.dxftype() == 'LINE':
                                start_point = entity.dxf.start
                                end_point = entity.dxf.end
                                gcode_file.write(f"G1 X{end_point.x:.{accuracy}f} Y{end_point.y:.{accuracy}f}\n")
                            elif entity.dxftype() == 'ARC':
                                center = entity.dxf.center
                                i=sorted_entities.index(entity)
                                radius = entity.dxf.radius
                                start = arranged_lines[i][0]
                                end = arranged_lines[i][1]
                                start_angle = entity.dxf.start_angle
                                end_angle = entity.dxf.end_angle
                                direction = self.determine_arc_direction(center, start, end)
                                s_x = entity.start_point[0]
                                s_y = entity.start_point[1]
                                e_x = entity.end_point[0]
                                e_y = entity.end_point[1]
                                # Ensure start_angle and end_angle are within 0 to 360 degrees range
                                start_angle = start_angle % 360
                                end_angle = end_angle % 360
                                angle_x = 360-(start_angle-end_angle)
                                r=radius
                                # Calculate the angle span of the arc
                                angle_span = end_angle - start_angle
                                if angle_span < 0:
                                    angle_span += 360
                                # Check if the arc spans more than 180 degrees
                                if angle_span > 180:
                                    r = radius*(-1)
                                # Calculate start and end points using arc formulas
                                start_angle_rad = math.radians(start_angle)
                                end_angle_rad = math.radians(end_angle)
                                start_x = arranged_lines[i][0][0]
                                start_y = arranged_lines[i][0][1]
                                end_x = arranged_lines[i][1][0]
                                end_y = arranged_lines[i][1][1]
                                if direction=='clockwise' and (start_x!=round(s_x,2) or start_y!=round(s_y,2)):
                                    gcode_file.write(f"G2 X{end_x:.{accuracy}f} Y{end_y:.{accuracy}f} R{r:.{accuracy}f} F{feed_rate}\n")
                                else:
                                    gcode_file.write(f"G3 X{end_x:.{accuracy}f} Y{end_y:.{accuracy}f} R{r:.{accuracy}f} F{feed_rate}\n")


                            elif entity.dxftype() == 'CIRCLE':
                                center = entity.dxf.center
                                radius = entity.dxf.radius
                                gcode_file.write(f"G0 X{center[0]:.{accuracy}f} Y{center[1]:.{accuracy}f}\n")
                                gcode_file.write(f"G1 X{center[0]-radius:.{accuracy}f} Y{center[1]:.{accuracy}f}\n")
                                gcode_file.write(f"G2 X{center[0]-radius:.{accuracy}f} Y{center[1]:.{accuracy}f} I{radius:.{accuracy}f} J{0:.{accuracy}f} F{feed_rate}\n")
                            elif entity.dxftype() == 'SPLINE':
                                control_points = entity.control_points
                                control_points = np.array([i[:-1] for i in control_points])
                                degree = entity.dxf.degree
                                knots = entity.knots
                                np.sort(knots)
                                spline = BSpline(knots, control_points, degree)
                                t = np.linspace(knots[degree], knots[-degree], 100)
                                spline_points = spline(t)
                                simplified_points = rdp(spline_points, epsilon=10**(saccuracy*-1))
                                x, y = simplified_points[:, 0], simplified_points[:, 1]
                                for i in range(len(x) - 1):
                                    x1, y1 = x[i], y[i]
                                    x2, y2 = x[i + 1], y[i + 1]
                                    gcode_line = f"G1 X{x2:.{accuracy}f} Y{y2:.{accuracy}f}\n"
                                    gcode_file.write(gcode_line)
                    gcode_file.write("M05\n")
                gcode_file.write("G53 G0 X0 Y0\n")
                gcode_file.write("M30\n")