import sys
import collections
import time
import math

# --- ТУТ ЛАГОДЖУ ПАЙТОН ---
# Це технічна штука, щоб старі бібліотеки для дрона дружили з новою версією Python.
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Mapping = collections.abc.Mapping
    collections.Sequence = collections.abc.Sequence
    collections.Iterable = collections.abc.Iterable
    collections.Callable = collections.abc.Callable

from dronekit import connect, VehicleMode, LocationGlobalRelative

# --- КУДИ МИ ЛЕТИМО (ТОЧКА Б) ---
TARGET_LAT = 50.443326  # Широта (де саме в Києві ми маємо бути)
TARGET_LON = 30.448078  # Довгота
TARGET_ALT = 300.0      # Висота, на якій дрон буде летіти (300 метрів)

# --- НАЛАШТУВАННЯ «ХАРАКТЕРУ» ДРОНА ---
MAX_SPEED = 12.0      # Швидше за цю позначку (12 м/с) дрон не розганятиметься
KP_POS = 0.6          # Чим далі ми від цілі, тим швидше дрон намагається до неї бігти
KP_VEL = 45.0         # Наскільки різко дрон реагує, якщо швидкість відрізняється від потрібної
KI_VEL = 0.18         # Ця штука допомагає дрону напирати сильніше, якщо вітер його зносить
P_ALT = 25            # Сила, з якою дрон тримає висоту, щоб не просідати
RC_NEUTRAL = 1500     # Середнє положення джойстика (коли ми нічого не чіпаємо)

# Змінні, де дрон запам’ятовує, наскільки сильно його знесло вітром
i_err_n = 0
i_err_e = 0

# --- ПІДКЛЮЧЕННЯ ---
print("Підключення до транспортного засобу...")
# Коннектимося до симулятора за адресою, яку видає Mission Planner
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def set_param_safe(name, value):
    """Просто міняємо налаштування в самому дроні (наприклад, вітер)"""
    try:
        vehicle.parameters[name] = value
        time.sleep(0.3)
    except Exception as e:
        print(f"Не вдалося змінити налаштування: {e}")

def send_rc_control(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    """Ця функція ніби замість нас рухає стіки на пульті керування"""
    vehicle.channels.overrides = {
        '1': int(roll),      # Нахил вліво-вправо
        '2': int(pitch),     # Нахил вперед-назад
        '3': int(throttle),  # Газ (вгору-вниз)
        '4': int(yaw)        # Поворот навколо своєї осі
    }

def get_velocity_control(current_target_alt):
    """
    Тут ми рахуємо, як сильно треба нахилити дрон, 
    щоб він летів куди треба і не боявся вітру.
    """
    global i_err_n, i_err_e
    
    # Питаємо у дрона: Де ти зараз? та Як швидко летиш?
    curr_loc = vehicle.location.global_relative_frame
    v_n, v_e, v_d = vehicle.velocity

    # Рахуємо відстань до цілі в метрах (переводимо координати в зрозумілі метри)
    err_lat = TARGET_LAT - curr_loc.lat
    err_lon = TARGET_LON - curr_loc.lon
    dist_n = err_lat * 111320
    dist_e = err_lon * 111320 * math.cos(math.radians(curr_loc.lat))
    total_dist = math.sqrt(dist_n**2 + dist_e**2)

    # 1. Вирішуємо, з якою швидкістю хочемо летіти (чим ближче ціль, тим повільніше)
    target_v_n = max(min(dist_n * KP_POS, MAX_SPEED), -MAX_SPEED)
    target_v_e = max(min(dist_e * KP_POS, MAX_SPEED), -MAX_SPEED)

    # 2. Дивимося на різницю: як ми ХОЧЕМО летіти і як дрон летить НАСПРАВДІ
    err_v_n = target_v_n - v_n
    err_v_e = target_v_e - v_e

    # 3. Боремося з вітром, накопичуємо помилку. Якщо дрон бачить, що його 
    # постійно зносить, він починає нахилятися все сильніше і сильніше проти вітру.
    i_err_n += err_v_n * 0.2
    i_err_e += err_v_e * 0.2
    
    # Ставимо запобіжник, щоб дрон не зійшов з розуму від занадто великих розрахунків
    i_err_n = max(min(i_err_n, 500), -500)
    i_err_e = max(min(i_err_e, 500), -500)

    # 4. Рахуємо фінальні команди для пульта (наскільки сильно тиснути вперед і вбік)
    pitch_val = RC_NEUTRAL - (err_v_n * KP_VEL) - (i_err_n * KI_VEL)
    roll_val = RC_NEUTRAL + (err_v_e * KP_VEL) + (i_err_e * KI_VEL)
    
    # 5. Підтримуємо висоту (якщо дрон нижче цілі — додаємо газу, якщо вище — прибираємо)
    alt_err = current_target_alt - curr_loc.alt
    throttle_val = RC_NEUTRAL + (alt_err * P_ALT)

    # Обмежуємо нахили, щоб дрон не перекинувся в повітрі (не більше 25%)
    pitch_val = max(min(pitch_val, 1750), 1250)
    roll_val = max(min(roll_val, 1750), 1250)
    throttle_val = max(min(throttle_val, 1850), 1250)

    return roll_val, pitch_val, throttle_val, total_dist

def arm_and_takeoff(target_altitude):
    """Готуємо дрон, вмикаємо вітер і злітаємо в небо"""
    print("Перевірки ...")
    while not vehicle.is_armable:
        time.sleep(1)
    
    print("Налаштування середовища...")
    # Вмикаємо вітер 5 м/с, щоб перевірити, чи впорається наш код
    set_param_safe('SIM_WIND_SPD', 5)
    set_param_safe('SIM_WIND_DIR', 30)

    print("Ввімкнення двигунів...")
    # Ставимо автоматичний режим і запускаємо мотори
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    print(f"Зліт до {target_altitude}m...")
    # Команда на зліт
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        # Чекаємо, поки дрон підніметься майже на саму гору
        if alt >= target_altitude * 0.95:
            print("Висота досягнута!")
            break
        time.sleep(1)

# --- САМ ПРОЦЕС ПОЛЬОТУ ---
try:
    # Спочатку злітаємо
    arm_and_takeoff(TARGET_ALT)

    print("Перемикання в режим СТАБІЛІЗАЦІЇ ...")
    # Переходимо в режим, де наш скрипт повністю керує руками (стіками)
    vehicle.mode = VehicleMode("STABILIZE")
    
    current_target_alt = TARGET_ALT  # Починаємо з висоти 300 метрів
    phase = "APPROACH"              # Фаза: «Наближення»

    while True:
        # Рахуємо, що треба робити прямо зараз
        roll, pitch, throttle, dist = get_velocity_control(current_target_alt)

        # Якщо ми ще далеко — просто летимо в бік точки Б
        if phase == "APPROACH":
            if dist <= 4.5:  # Коли до цілі залишилося 4.5 метри...
                print(">>> Цільова зона досягнута! Початок точного спуску... <<<")
                phase = "DESCENT"  # ...перемикаємося на фазу Посадка
        
        # Під час посадки ми плавно зменшуємо висоту, поки дрон все ще бореться з вітром
        elif phase == "DESCENT":
            # Кожні 0.2 секунди ми просимо дрон бути на 0.8 метра нижче
            current_target_alt -= 0.8
            if current_target_alt < 0:
                current_target_alt = 0
            
            # Якщо ми вже біля самої землі (0.8 м) — вимикаємося
            if vehicle.location.global_relative_frame.alt <= 0.8:
                print(">>> Приземлення! Зупинка двигунів... <<<")
                break

        # Відправляємо наші команди на дрон
        send_rc_control(roll=roll, pitch=pitch, throttle=throttle, yaw=1500)

        # Виводимо на екран, де ми і що робимо
        print(f"[{phase}] Дистанція до цілі: {dist:.1f}m | Реальна висота: {vehicle.location.global_relative_frame.alt:.1f}m | Бажана висота: {current_target_alt:.1f}m")
        
        # Повторюємо це 5 разів на секунду
        time.sleep(0.2)

    # Коли все закінчили — сідаємо і глушимо мотори
    print("Місія виконана. Посадка ...")
    send_rc_control(1500, 1500, 1500, 1500)
    vehicle.mode = VehicleMode("LAND")
    
    while vehicle.armed:
        time.sleep(2)
        print("Очікування...")

except KeyboardInterrupt:
    # Якщо ми натиснули Ctrl+C — зупиняємо все терміново
    print("Перервано користувачем!")
finally:
    # На всяк випадок «відпускаємо» керування і закриваємо програму
    vehicle.channels.overrides = {}
    vehicle.close()
    print("Готово.")