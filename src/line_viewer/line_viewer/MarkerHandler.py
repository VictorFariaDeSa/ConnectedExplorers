from visualization_msgs.msg import Marker



class MarkerHandler:
    def __init__(self,max_safe_distance,min_safe_distance,line_alpha,line_scale):
        self.max_safe_distance = max_safe_distance
        self.min_safe_distance = min_safe_distance
        self.line_alpha = line_alpha
        self.line_scale = line_scale




    def get_marker_color(self,score):
        if score == 0:
            rgb_color = (1.0, 0.0, 0.0)
        else:         
            r_val = 1.0 - score
            g_val = 1.0 
            b_val = 0.0
            rgb_color = (r_val, g_val, b_val)
        return rgb_color

    def create_marker(self,point1, point2, marker_id, rgb_color, ref_frame, namespace, timestamp):
        marker = Marker()
        marker.header.frame_id = ref_frame
        marker.header.stamp = timestamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.color.r, marker.color.g, marker.color.b = rgb_color
        marker.color.a = float(self.line_alpha)
        marker.scale.x = float(self.line_scale)
        
        marker.points = [point1, point2]
        return marker