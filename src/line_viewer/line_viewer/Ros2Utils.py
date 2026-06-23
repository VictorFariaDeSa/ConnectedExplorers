from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np


def numpy_matrix_to_float64multArray(numpy_matrix:np.ndarray):
        msg = Float64MultiArray()
        msg.data = numpy_matrix.flatten().tolist()
        rows = numpy_matrix.shape[0]
        cols = numpy_matrix.shape[1]
        dim_rows = MultiArrayDimension()
        dim_rows.label = "rows"
        dim_rows.size = rows
        dim_rows.stride = rows * cols
        dim_cols = MultiArrayDimension()
        dim_cols.label = "cols"
        dim_cols.size = cols
        dim_cols.stride = cols
        msg.layout.dim = [dim_rows, dim_cols]
        return msg


def float64multArray_to_numpy_matrix(msg: Float64MultiArray) -> np.ndarray:
        if not msg.data:
            return np.array([])
        data = np.array(msg.data)
        if msg.layout.dim:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
            if rows * cols == len(data):
                return data.reshape((rows, cols))
        side_length = int(np.sqrt(len(data)))
        
        if side_length * side_length == len(data):
            return data.reshape((side_length, side_length))
        return data