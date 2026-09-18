#!/usr/bin/env python3
"""จำลอง Delivery FSM โดยใช้เฉพาะ Arduino #2, Keypad, LCD และ IR."""

import logging
import os
import time

import serial

from config import SERIAL_BAUD, SERIAL_TIMEOUT, SHELF_SERIAL_PORT
from shelf_client import ShelfClient


class ShelfFsmTest:
    def __init__(self, shelf: ShelfClient) -> None:
        self.shelf = shelf
        self.orders: list[tuple[int, int]] = []

    def run(self) -> None:
        print("Arduino #2 FSM simulation started. Press Ctrl+C to stop.")

        while True:
            self.orders.clear()

            while len(self.orders) < 2:
                shelf_number = self.select_shelf()
                if shelf_number is None:
                    self.reset_screen()
                    break

                if not self.wait_for_food(shelf_number):
                    self.reset_screen()
                    break

                table_number = self.select_table(shelf_number)
                if table_number is None:
                    self.reset_screen()
                    break

                self.orders.append((shelf_number, table_number))

                action = self.show_order_list()
                if action == "START":
                    self.simulate_delivery()
                    break
                if action == "RESET":
                    self.reset_screen()
                    break

    def select_shelf(self) -> int | None:
        while True:
            self.shelf.lcd_print(0, "Select shelf 1/2")
            self.shelf.lcd_print(1, "Choose then B")

            key = self.shelf.wait_for_key(valid_keys=["1", "2", "*"])
            if key == "*":
                return None

            selected = int(key)
            if any(order[0] == selected for order in self.orders):
                self.shelf.lcd_print(0, "Shelf already used")
                self.shelf.lcd_print(1, "Choose another")
                time.sleep(1.2)
                continue

            self.shelf.lcd_print(0, f"Shelf {selected} selected")
            self.shelf.lcd_print(1, "B=OK  *=Reset")

            confirm = self.shelf.wait_for_key(valid_keys=["B", "*"])
            if confirm == "B":
                return selected
            return None

    def wait_for_food(self, shelf_number: int) -> bool:
        self.shelf.lcd_print(0, f"Place food S{shelf_number}")
        self.shelf.lcd_print(1, "Waiting... *=RST")

        while True:
            if self.shelf.ir_has_food(shelf_number):
                self.shelf.lcd_print(0, "Food detected")
                self.shelf.lcd_print(1, f"Shelf {shelf_number} ready")
                time.sleep(0.8)
                return True

            if self.shelf.poll_key() == "*":
                return False

            time.sleep(0.05)

    def select_table(self, shelf_number: int) -> int | None:
        self.shelf.lcd_print(0, f"Table for shelf {shelf_number}")
        self.shelf.lcd_print(1, "Choose 1 or 2")

        table_key = self.shelf.wait_for_key(valid_keys=["1", "2", "*"])
        if table_key == "*":
            return None

        table_number = int(table_key)
        self.shelf.lcd_print(0, f"Table {table_number}")
        self.shelf.lcd_print(1, "C=OK  *=Reset")

        confirm = self.shelf.wait_for_key(valid_keys=["C", "*"])
        if confirm == "C":
            return table_number
        return None

    def show_order_list(self) -> str:
        first = self.orders[0]
        text = f"S{first[0]}>T{first[1]}"

        if len(self.orders) == 2:
            second = self.orders[1]
            text += f" S{second[0]}>T{second[1]}"

        self.shelf.lcd_print(0, text)

        if len(self.orders) < 2:
            self.shelf.lcd_print(1, "A=Add #=Start")
            key = self.shelf.wait_for_key(valid_keys=["A", "#", "*"])
            if key == "A":
                return "ADD"
        else:
            self.shelf.lcd_print(1, "#=Start *=Reset")
            key = self.shelf.wait_for_key(valid_keys=["#", "*"])

        return "START" if key == "#" else "RESET"

    def simulate_delivery(self) -> None:
        for shelf_number, table_number in self.orders:
            self.shelf.lcd_print(0, f"Going to table {table_number}")
            self.shelf.lcd_print(1, f"Food shelf {shelf_number}")
            print(f"Simulating travel: shelf {shelf_number} -> table {table_number}")
            time.sleep(2.0)

            self.shelf.lcd_print(0, f"Arrived table {table_number}")
            self.shelf.lcd_print(1, f"Take food S{shelf_number}")

            self.shelf.wait_for_food_removed(shelf_number)
            self.shelf.lcd_print(0, "Food received")
            self.shelf.lcd_print(1, "Thank you")
            time.sleep(1.2)

        self.shelf.lcd_print(0, "All delivered")
        self.shelf.lcd_print(1, "Returning home")
        print("All simulated deliveries completed. Returning home.")
        time.sleep(2.0)

        self.shelf.lcd_print(0, "Arrived station")
        self.shelf.lcd_print(1, "Ready for next")
        time.sleep(1.2)

    def reset_screen(self) -> None:
        self.orders.clear()
        self.shelf.lcd_print(0, "RESET DONE")
        self.shelf.lcd_print(1, "Starting again")
        time.sleep(1.0)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)-8s  %(message)s",
        datefmt="%H:%M:%S",
    )

    port = os.environ.get("SHELF_PORT", SHELF_SERIAL_PORT)
    print(f"Connecting Arduino #2: {port} @ {SERIAL_BAUD}")

    connection = serial.Serial(port, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
    time.sleep(2.0)  # Arduino Uno จะรีเซตเมื่อเปิด Serial

    shelf = ShelfClient(connection)
    shelf.start()

    try:
        ShelfFsmTest(shelf).run()
    except KeyboardInterrupt:
        print("\nStopping Arduino #2 FSM simulation.")
    finally:
        shelf.stop()
        connection.close()


if __name__ == "__main__":
    main()
