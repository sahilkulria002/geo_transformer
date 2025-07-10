#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geo_transformer.srv import SetOrigin, GetOrigin, FromLL, ToLL

class ServiceTester(Node):
    def __init__(self):
        super().__init__('geo_service_tester')
        self.origin = [28.6139, 77.2090, 0.0]  # India Gate
        self.test_point = [28.6145, 77.2105, 0.0]  # Nearby point

        # Use the correct service names as advertised by the C++ node
        self.set_origin_client = self.create_client(SetOrigin, '/local_coordinate/set')
        self.get_origin_client = self.create_client(GetOrigin, '/local_coordinate/get')
        self.from_ll_client = self.create_client(FromLL, '/from_ll')
        self.to_ll_client = self.create_client(ToLL, '/to_ll')
        # Named origin services
        from geo_transformer.srv import AddNamedOrigin, SwitchOrigin, ListOrigins, GetCurrentOriginName
        self.add_named_origin_client = self.create_client(AddNamedOrigin, '/origin_store/add_named_origin')
        self.switch_origin_client = self.create_client(SwitchOrigin, '/origin_store/switch_origin')
        self.list_origins_client = self.create_client(ListOrigins, '/origin_store/list_origins')
        self.get_current_origin_name_client = self.create_client(GetCurrentOriginName, '/origin_store/get_current_origin_name')

        self.timer = self.create_timer(1.0, self.run_sequence)

    # Overwrite timer to start with named origin tests
    def run_sequence(self):
        self.timer.cancel()
        self.test_add_named_origin()

    # --- Named Origin Service Tests ---
    def test_add_named_origin(self):
        print("[AddNamedOrigin] Waiting for service...")
        self.add_named_origin_client.wait_for_service()
        req = self.add_named_origin_client.srv_type.Request()
        req.name = "test_origin"
        req.latitude = 28.6150
        req.longitude = 77.2110
        req.altitude = 10.0
        self.add_named_origin_client.call_async(req).add_done_callback(self.on_add_named_origin)

    def on_add_named_origin(self, future):
        res = future.result()
        print(f"[AddNamedOrigin] success={res.success}, message={res.message}")
        self.test_switch_origin()

    def test_switch_origin(self):
        print("[SwitchOrigin] Waiting for service...")
        self.switch_origin_client.wait_for_service()
        req = self.switch_origin_client.srv_type.Request()
        req.name = "test_origin"
        self.switch_origin_client.call_async(req).add_done_callback(self.on_switch_origin)

    def on_switch_origin(self, future):
        res = future.result()
        print(f"[SwitchOrigin] success={res.success}, message={res.message}")
        self.test_list_origins()

    def test_list_origins(self):
        print("[ListOrigins] Waiting for service...")
        self.list_origins_client.wait_for_service()
        req = self.list_origins_client.srv_type.Request()
        self.list_origins_client.call_async(req).add_done_callback(self.on_list_origins)

    def on_list_origins(self, future):
        res = future.result()
        print(f"[ListOrigins] names={res.names}")
        self.test_get_current_origin_name()

    def test_get_current_origin_name(self):
        print("[GetCurrentOriginName] Waiting for service...")
        self.get_current_origin_name_client.wait_for_service()
        req = self.get_current_origin_name_client.srv_type.Request()
        self.get_current_origin_name_client.call_async(req).add_done_callback(self.on_get_current_origin_name)

    def on_get_current_origin_name(self, future):
        res = future.result()
        print(f"[GetCurrentOriginName] name={res.name}")
        # After named origin tests, continue with main sequence
        self.run_main_sequence()

    def run_main_sequence(self):
        print("[SetOrigin] Waiting for service...")
        self.set_origin_client.wait_for_service()
        req = SetOrigin.Request()
        req.latitude, req.longitude, req.altitude = self.origin
        self.set_origin_client.call_async(req).add_done_callback(self.on_set_origin)

    def on_set_origin(self, future):
        print("[SetOrigin] Done")

        print("[GetOrigin] Waiting for service...")
        self.get_origin_client.wait_for_service()
        self.get_origin_client.call_async(GetOrigin.Request()).add_done_callback(self.on_get_origin)

    def on_get_origin(self, future):
        res = future.result()
        print(f"[GetOrigin] lat={res.latitude}, lon={res.longitude}, alt={res.altitude}")

        # Also get current origin name
        self.get_current_origin_name_client.wait_for_service()
        req_name = self.get_current_origin_name_client.srv_type.Request()
        future_name = self.get_current_origin_name_client.call_async(req_name)
        future_name.add_done_callback(lambda f: self.start_batch_and_single_from_ll(res, f.result().name))

    def start_batch_and_single_from_ll(self, origin_res, origin_name):
        # Single-point test (as before)
        print("[FromLL] Waiting for service (single point)...")
        self.from_ll_client.wait_for_service()
        req = FromLL.Request()
        req.latitude = [self.test_point[0]]
        req.longitude = [self.test_point[1]]
        req.altitude = [self.test_point[2]]
        self.from_ll_client.call_async(req).add_done_callback(lambda f: self.on_from_ll(f, origin_name, origin_res))

        # Batch test: 3 points
        self.batch_points = [
            [28.6145, 77.2105, 0.0],
            [28.6150, 77.2110, 10.0],
            [28.6160, 77.2120, 20.0]
        ]
        batch_req = FromLL.Request()
        batch_req.latitude = [p[0] for p in self.batch_points]
        batch_req.longitude = [p[1] for p in self.batch_points]
        batch_req.altitude = [p[2] for p in self.batch_points]
        self.from_ll_client.call_async(batch_req).add_done_callback(lambda f: self.on_from_ll_batch(f, origin_name, origin_res))

    def on_from_ll(self, future, origin_name, origin_res):
        res = future.result()
        # Print current origin info
        print(f"[FromLL] (Current Origin: name='{origin_name}', lat={origin_res.latitude}, lon={origin_res.longitude}, alt={origin_res.altitude})")
        print(f"[FromLL] x={res.x[0]}, y={res.y[0]}, z={res.z[0]}")

        print("[ToLL] Waiting for service (single point)...")
        self.to_ll_client.wait_for_service()
        req = ToLL.Request()
        req.x = [res.x[0]]
        req.y = [res.y[0]]
        req.z = [res.z[0]]
        self.to_ll_client.call_async(req).add_done_callback(lambda f: self.on_to_ll(f, origin_name, origin_res))

    def on_from_ll_batch(self, future, origin_name, origin_res):
        res = future.result()
        print(f"[FromLL-Batch] (Current Origin: name='{origin_name}', lat={origin_res.latitude}, lon={origin_res.longitude}, alt={origin_res.altitude})")
        for i, (x, y, z) in enumerate(zip(res.x, res.y, res.z)):
            print(f"[FromLL-Batch] Point {i}: x={x}, y={y}, z={z}")
        # Now test ToLL batch
        print("[ToLL] Waiting for service (batch)...")
        self.to_ll_client.wait_for_service()
        req = ToLL.Request()
        req.x = res.x
        req.y = res.y
        req.z = res.z
        self.to_ll_client.call_async(req).add_done_callback(lambda f: self.on_to_ll_batch(f, origin_name, origin_res))

    def on_to_ll(self, future, origin_name, origin_res):
        res = future.result()
        # Print current origin info
        print(f"[ToLL] (Current Origin: name='{origin_name}', lat={origin_res.latitude}, lon={origin_res.longitude}, alt={origin_res.altitude})")
        print(f"[ToLL] lat={res.latitude[0]}, lon={res.longitude[0]}, alt={res.altitude[0]}")

        # Round-trip check: compare input test_point with output from ToLL
        import math
        lat1, lon1, alt1 = self.test_point
        lat2, lon2, alt2 = res.latitude[0], res.longitude[0], res.altitude[0]

        # Compute differences
        dlat = abs(lat1 - lat2)
        dlon = abs(lon1 - lon2)
        dalt = abs(alt1 - alt2)

        # Haversine formula for distance in meters (ignoring altitude)
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        horizontal_error = R * c
        vertical_error = dalt

        print(f"[RoundTrip] Horizontal error: {horizontal_error:.6f} m, Vertical error: {vertical_error:.6f} m")
        # Optionally, assert or warn if error is too large
        if horizontal_error < 1.0 and vertical_error < 0.1:
            print("[RoundTrip] ✅ Transformation is accurate (within 1 meter, 0.1m altitude)")
        else:
            print("[RoundTrip] ❌ Transformation error is too large!")
        # Do not shutdown here, wait for batch test to finish

    def on_to_ll_batch(self, future, origin_name, origin_res):
        res = future.result()
        print(f"[ToLL-Batch] (Current Origin: name='{origin_name}', lat={origin_res.latitude}, lon={origin_res.longitude}, alt={origin_res.altitude})")
        for i, (lat, lon, alt) in enumerate(zip(res.latitude, res.longitude, res.altitude)):
            print(f"[ToLL-Batch] Point {i}: lat={lat}, lon={lon}, alt={alt}")
        print("[Batch Test Complete]")
        rclpy.shutdown()


def main():
    print("Starting service tester...")
    rclpy.init()
    rclpy.spin(ServiceTester())

if __name__ == '__main__':
    main()
