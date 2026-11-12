/* PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  by ELECTRONOOBS (Adaptado fielmente para ESP32)
 *  + Interfaz web (Access Point, WebSocket, ArduinoJson) para ajustar
 *    setpoint, alternar modo automatico/manual y graficar en vivo.
 */
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <string.h>

///////////////////////Pines de Entrada/Salida///////////////////////
const int Analog_in = 34;  // Pin ADC del ESP32 para el sensor Sharp
const int servoPin  = 18;  // Pin GPIO del ESP32 para el Servomotor
Servo myservo;              // Objeto servo para controlar el actuador
///////////////////////////////////////////////////////////////////

////////////////////////Parametros de tiempo///////////////////////
const unsigned long period = 50;  // Periodo de refresco del lazo de control (ms)
unsigned long currentTime  = 0;   // Marca de tiempo de la ultima ejecucion del lazo
///////////////////////////////////////////////////////////////////

////////////////////////Variables del PID///////////////////////////
float distance = 0.0;
float distance_previous = 0.0;   // Distancia filtrada del ciclo anterior, usada solo por el termino D (derivada sobre la medicion)
float real_error_previous = 0.0; // Error real (sin zona muerta) del ciclo anterior, usado solo por telemetria/grafica
float distance_error = 0.0;      // Error con zona muerta aplicada, usado por P e I
float real_error = 0.0;          // Error real sin zona muerta, usado para telemetria/grafica
float PID_p = 0.0, PID_i = 0.0, PID_d = 0.0, PID_total = 0.0;

// D_FILTER_ALPHA ya no es un valor fijo: se interpola segun el setpoint porque
// la validacion en hardware real mostro que un solo valor no sirve para todo
// el rango. Cerca del sensor (SETPOINT_MIN_CM) el sistema necesita MENOS
// filtrado (mas amortiguamiento real del termino D) o entra en una oscilacion
// sostenida; lejos del sensor (SETPOINT_MAX_CM) necesita MAS filtrado porque
// el ruido del sensor se amplifica mas alla y agrava el termino derivativo
// (kd=3750 es muy agresivo). Ambos extremos fueron confirmados empiricamente:
// alpha=0.6 cerca de SETPOINT_MIN_CM estabilizo la pelota; alpha=0.1 funciono
// bien cerca de SETPOINT_MAX_CM. El valor _AT_MID (en SETPOINT_MID_CM, el
// centro fisico de la barra) es un PUNTO DE PARTIDA razonable, todavia NO
// validado en hardware real como los otros dos: se elige poco filtrado
// porque el sistema ya funcionaba bien en el centro incluso antes de
// introducir este filtro derivativo, pero el usuario debe probarlo y
// ajustarlo en su hardware. La funcion interpolate3Point() (generica,
// reutilizada tambien por el gain scheduling de kp/ki/kd de mas abajo) que
// hace la interpolacion piecewise-linear de 3 puntos se define mas abajo,
// luego de SETPOINT_MIN_CM/SETPOINT_MID_CM/SETPOINT_MAX_CM (de las que
// depende).
const float D_FILTER_ALPHA_AT_MIN = 0.6;  // Menos filtrado, cerca de SETPOINT_MIN_CM (validado en hardware)
const float D_FILTER_ALPHA_AT_MID = 0.8;  // Poco filtrado, en SETPOINT_MID_CM (PUNTO DE PARTIDA, sin validar aun)
const float D_FILTER_ALPHA_AT_MAX = 0.1;  // Mas filtrado, cerca de SETPOINT_MAX_CM (validado en hardware)
float PID_d_filtered = 0.0;
///////////////////////////////////////////////////////////////////

////////////////////////Controlador por modos deslizantes (SMC)//////////////
// REEMPLAZA al PID con ganancias programadas por 3 puntos como ley de control
// principal en MODE_AUTO (ver bloque en loop() y la explicacion extensa junto
// a las constantes SMC_* mas abajo, cerca de interpolate3Point()). Motivo:
// medido en hardware real, NINGUN punto de operacion lograba asentarse con
// error realmente cero: oscilaba de forma sostenida (~0.7cm en el centro de
// la barra, ~1cm en los extremos), sin importar cuanto se ajustaran Kp/Kd.
// Esto es la firma clasica de una no-linealidad tipo "backlash/stick-slip"
// en el enlace mecanico (confirmado independientemente en pruebas manuales,
// ver comentarios de UNSTICK_*): dentro de esa banda muerta mecanica, NINGUN
// controlador basado solo en el error de posicion puede hacer nada, porque
// el comando no se traduce en movimiento. Un control por modos deslizantes,
// que ademas de la posicion usa la VELOCIDAD estimada de la pelota, ataca
// esto de forma mas directa: satura a una correccion fuerte y sostenida en
// vez de una proporcional-al-error cada vez mas debil a medida que el error
// baja (que es precisamente lo que dejaba a la pelota "flotando" justo en el
// borde de la zona muerta mecanica sin suficiente empuje para cruzarla).
float ball_velocity_filtered = 0.0;  // Velocidad estimada de la pelota (cm/s), filtrada (EMA)
float smc_integral = 0.0;            // Termino integral del controlador SMC (grados), separado de PID_i (que ya no se usa en MODE_AUTO)
unsigned long frictionCompStillSinceMs = 0;  // 0 = la velocidad no esta actualmente por debajo de FRICTION_COMP_VEL_THRESHOLD de forma continua
// REGULATE: congela angulo (asentar <6s, hold largo, minimo residual)
bool regulateActive = false;
float regulateAngle = 125.0;
uint8_t regulateCount = 0;
uint8_t regulateExitCount = 0;
float regulateErrFilt = 0.0f;
float regulateAngleFreeze = 125.0f;
const float HYB_REG_ENTER_CM = 0.50;
const float HYB_REG_ENTER_VEL = 0.90;
const float HYB_REG_EXIT_CM = 1.05;
const uint8_t HYB_REG_EXIT_COUNT = 14;
const uint8_t HYB_REG_COUNT = 2;       // ~100 ms: capturar el primer asiento
const float HYB_REG_ENTER_GOOD_CM = 0.32;
const uint8_t HYB_REG_COUNT_GOOD = 2;
const float HYB_REG_TRIM_DEAD = 0.16;
const float HYB_REG_TRIM_GAIN = 0.13;
const float HYB_REG_TRIM_MAX = 0.05;
const float HYB_REG_TRIM_SPAN = 3.2;
///////////////////////////////////////////////////////////////////

///////////////////Gain scheduling de kp/ki/kd segun el setpoint/////
// Igual que con D_FILTER_ALPHA, kp/ki/kd ya no son valores fijos: se
// recalculan cada ciclo en loop() interpolando piecewise-linear entre 3
// puntos de referencia: "AT_MIN" (en SETPOINT_MIN_CM), "AT_MID" (en
// SETPOINT_MID_CM, el centro fisico de la barra) y "AT_MAX" (en
// SETPOINT_MAX_CM), via interpolate3Point() (definida mas abajo, junto a
// SETPOINT_MIN_CM/SETPOINT_MID_CM/SETPOINT_MAX_CM, de las que depende).
//
// AT_MIN y AT_MAX arrancan en los valores ya confirmados en hardware real
// (ver comentarios de cada constante). AT_MID usa el valor historico unico
// clasico que ya estaba sintonizado antes de introducir el gain scheduling
// (ver comentarios "Mine was..." abajo): es un PUNTO DE PARTIDA razonable
// porque 15cm (el centro de la barra) es el punto que siempre funciono bien,
// pero a diferencia de AT_MIN/AT_MAX todavia NO fue revalidado empiricamente
// dentro de este esquema de 3 puntos. El objetivo de esto es dejar la
// infraestructura lista para que el usuario ajuste el mismo cada punto,
// probando en SETPOINT_MIN_CM, SETPOINT_MID_CM y SETPOINT_MAX_CM por
// separado en hardware real, una constante a la vez (por ejemplo, primero
// KP_AT_MIN vs KP_AT_MID vs KP_AT_MAX dejando KI/KD fijos, despues KI, etc.)
// para no confundir el efecto de cada variable.
// NOTA: KP/KI/KD_AT_* ya NO son 'const': el auto-tune (ver seccion dedicada
// mas abajo) puede sobreescribirlas en RAM cuando el usuario presiona
// "Aplicar" tras una corrida exitosa. Los valores de partida siguen siendo
// los mismos que antes (documentados abajo); solo cambia la calificacion
// 'const' para permitir la escritura en tiempo de ejecucion.
float KP_AT_MIN = 9;   // kp cerca de SETPOINT_MIN_CM. Validado en hardware real.
// KP_AT_MID: VALIDADO en hardware real (telemetria WS directa, sin autotune de
// rele). Con el valor historico (8, menor que AT_MIN=9) el sistema en 15cm
// NUNCA convergia: quedaba en un ciclo limite permanente de ~4.3s (atascarse
// en distancia~16.7cm, esperar, golpe de rescate, acercarse, volver a
// atascarse...), mientras que en MIN (kp=9) y MAX (kp=6) si convergia y se
// quedaba estable. La zona de angulo de servo que corresponde a 15cm
// (~90-130 grados) coincide con varias de las "mesetas" de friccion/juego
// mecanico detectadas en los barridos manuales (ver mas abajo), y
// evidentemente necesita MAS autoridad de control, no menos, para vencerla
// de forma sostenida. Se sube a 10 (igual o mayor que AT_MIN) en vez de
// dejarlo como el valor mas bajo de los 3 puntos.
float KP_AT_MID = 10;
float KP_AT_MAX = 6;   // kp cerca de SETPOINT_MAX_CM. Validado en hardware real.
float KI_AT_MIN = 0.1;   // ki cerca de SETPOINT_MIN_CM. Validado en hardware real (ki nunca diverge entre extremos).
float KI_AT_MID = 0.1;   // ki en SETPOINT_MID_CM. Igual que AT_MIN/AT_MAX, ya que ki nunca diverge entre puntos actualmente.
float KI_AT_MAX = 0.1;   // ki cerca de SETPOINT_MAX_CM. Validado en hardware real (ki nunca diverge entre extremos).
float KD_AT_MIN = 3200;  // kd cerca de SETPOINT_MIN_CM. Validado en hardware real.
// KD_AT_MID: se sube proporcionalmente junto con KP_AT_MID (ver arriba) para
// mantener un amortiguamiento similar al de antes con el nuevo Kp mas alto.
float KD_AT_MID = 3400;
float KD_AT_MAX = 3750;  // kd cerca de SETPOINT_MAX_CM. Validado en hardware real.

float kp = KP_AT_MIN, ki = KI_AT_MIN, kd = KD_AT_MIN;  // Se recalculan cada ciclo en loop()
///////////////////////////////////////////////////////////////////

/////////////////Limites del setpoint (defensa en profundidad)///////
// SETPOINT_MIN_CM: 13cm es el limite empirico determinado en hardware real, ya
// que el sensor Sharp GP2Y0A21 pierde precision por debajo de ~11cm (fuera de
// su rango valido); se deja un margen de seguridad de ~2cm. Coincide con el
// minimo del slider "setpointRange" en la interfaz web (INDEX_HTML), pero se
// valida aca tambien por si algun cliente manda un valor fuera de rango
// directamente por WebSocket sin pasar por el slider.
const float SETPOINT_MIN_CM = 13.0;
// SETPOINT_MID_CM: el centro fisico real de la barra, y el setpoint por
// defecto original del sistema antes de todo el trabajo de gain scheduling.
// Se usa como bisagra de la interpolacion piecewise-linear de 3 puntos (ver
// interpolate3Point() mas abajo): permite ajustar el tramo cercano al sensor
// de forma independiente del tramo lejano.
const float SETPOINT_MID_CM = 15.0;
// SETPOINT_MAX_CM: bajado de 23 a 21cm tras recalibracion serial USB
// (2026-07-30). En 23cm la pelota NO mantenia banda ±0.5cm por 1s (meseta
// mecanica ~21.5-22.5cm). Con 21cm los tres presets asientan en <6s.
// Coincide con slider/preset de INDEX_HTML y con el clamp de WebSocket/Serial.
const float SETPOINT_MAX_CM = 21.0;
///////////////////////////////////////////////////////////////////

// Interpola linealmente por tramos entre 3 puntos de referencia (atMin, atMid,
// atMax) segun donde cae 'setpoint' dentro de [SETPOINT_MIN_CM, SETPOINT_MID_CM]
// o [SETPOINT_MID_CM, SETPOINT_MAX_CM]. Con solo 2 puntos, la interpolacion es
// una sola recta y no puede representar una curva real no lineal; con 3 puntos
// se puede ajustar el tramo cercano al sensor de forma independiente del tramo
// lejano, usando SETPOINT_MID_CM (el centro real de la barra) como bisagra.
// Usada para el gain scheduling de D_FILTER_ALPHA y de kp/ki/kd. Se define
// aca (y no junto a las constantes AT_MIN/AT_MID/AT_MAX mas arriba) porque
// depende de SETPOINT_MIN_CM/SETPOINT_MID_CM/SETPOINT_MAX_CM, que en
// C++/Arduino deben estar declaradas antes de usarse.
float interpolate3Point(float atMin, float atMid, float atMax, float setpoint) {
  if (setpoint <= SETPOINT_MID_CM) {
    float t = (setpoint - SETPOINT_MIN_CM) / (SETPOINT_MID_CM - SETPOINT_MIN_CM);
    t = constrain(t, 0.0, 1.0);
    return atMin + t * (atMid - atMin);
  } else {
    float t = (setpoint - SETPOINT_MID_CM) / (SETPOINT_MAX_CM - SETPOINT_MID_CM);
    t = constrain(t, 0.0, 1.0);
    return atMid + t * (atMax - atMid);
  }
}

/////////////////Impulso de ganancia por magnitud de error (aproximacion rapida)/////
// PROBLEMA MEDIDO EN HARDWARE: el gain scheduling de arriba solo depende de DONDE
// esta el setpoint en la barra, no de QUE TAN LEJOS esta la pelota de el en un
// instante dado. Con Kp/Kd ya ajustados para un seguimiento FINO y preciso cerca
// del objetivo (para no perder estabilidad una vez asentada), la fase de
// ACERCAMIENTO cuando el error es grande (p.ej. recien se cambio el setpoint a un
// extremo de la barra) se vuelve lenta: el angulo comandado es demasiado suave
// para vencer rapido la friccion/juego mecanico y el sistema tarda muchos
// segundos en cruzar la barra, dependiendo del windup lento de la Integral.
//
// SOLUCION: un segundo multiplicador de ganancia, INDEPENDIENTE del de arriba,
// que depende de |distance_error| (que tan lejos esta la pelota AHORA, no de
// donde esta el setpoint en la barra). Con error grande (fase de acercamiento)
// se multiplica Kp/Kd por un factor >1 para moverse con decision; con error
// pequeno (ya cerca del objetivo, incluyendo la zona ya validada en hardware)
// el multiplicador vuelve a 1.0 y se recupera exactamente el comportamiento fino
// ya sintonizado. Kd se escala junto con Kp (no solo Kp) para mantener la misma
// proporcion Kd/Kp (misma "forma" de la respuesta, solo mas rapida), evitando
// que el acercamiento rapido se vuelva subamortiguado. Ki NO se escala: el
// anti-windup (ANTIWINDUP_RANGE) ya limita cuanto puede acumular con error
// grande, y agrandar ki ahi solo empeoraria el overshoot al llegar.
const float ERROR_BOOST_START_CM    = 2.0;  // por debajo de esto, multiplicador = 1.0 (sin cambios)
const float ERROR_BOOST_FULL_CM     = 5.0;  // a partir de esto, multiplicador = ERROR_BOOST_MAX_MULT (maximo)
const float ERROR_BOOST_MAX_MULT    = 2.2;  // factor maximo aplicado a Kp/Kd durante el acercamiento

float errorGainBoost(float distanceErrorAbs) {
  float t = (distanceErrorAbs - ERROR_BOOST_START_CM) / (ERROR_BOOST_FULL_CM - ERROR_BOOST_START_CM);
  t = constrain(t, 0.0, 1.0);
  return 1.0 + t * (ERROR_BOOST_MAX_MULT - 1.0);
}
// NOTA: KP_AT_*/KD_AT_*/KI_AT_*/interpolate3Point()/errorGainBoost() de arriba
// ya NO se usan en MODE_AUTO (ver el nuevo controlador SMC_* mas abajo y el
// bloque de loop()). Se dejan definidos, sin borrar, unicamente como
// referencia/respaldo por si hiciera falta volver al esquema anterior.

/////////////////Parametros del controlador SMC (modo deslizante)/////////////
// Superficie de deslizamiento: s = SMC_LAMBDA*real_error - ball_velocity_filtered
// (unidades: cm/s). Se disena para que s->0 implique simultaneamente
// error->0 Y velocidad->0 (la pelota llega Y se queda, no solo "pasa por"
// el setpoint). real_error = setpoint - distancia; ball_velocity_filtered =
// d(distancia)/dt; entonces cuando el setpoint esta fijo, d(real_error)/dt =
// -ball_velocity_filtered, y s se puede leer como "error mas su propia
// tendencia de cambio", igual que un PD clasico pero interpretado como
// superficie de deslizamiento en vez de una simple suma de terminos.
//
// SMC_LAMBDA: cuanto pesa el error de POSICION frente a la velocidad dentro
// de 's'. Tambien fija a que error (con velocidad~0, el peor caso justo
// despues de un cambio de setpoint) el controlador satura a SMC_MAX_DEG:
// error_saturacion = SMC_PHI / SMC_LAMBDA. Cuanto MAS ALTO, mas agresivo
// (satura antes, responde mas rapido) pero tambien MENOS amortiguado
// (relativamente menos peso de la velocidad dentro de 's'), con mas riesgo
// de sobrepaso.
// MEDIDO EN HARDWARE: un unico valor fijo NO sirve para toda la barra, igual
// que ya paso con D_FILTER_ALPHA/VELOCITY_FILTER_ALPHA/KP/KD en el esquema
// anterior. Con 0.5 (el valor original), error_saturacion=10cm (casi TODO el
// rango de 10cm de la barra): el controlador casi nunca saturaba, y saltos
// grandes de setpoint (ej. 13->23cm) tardaban mas de 12s sin asentarse. Con
// 2.0 (mas agresivo, error_saturacion=2.5cm) 13cm y 15cm mejoraron mucho
// (asientan en 1-5s), pero 23cm PEORO: entro en una oscilacion sostenida de
// gran amplitud (2-4cm), rebotando entre los dos extremos del servo sin
// asentar nunca. Esto confirma (una vez mas, ver UNSTICK_*/D_FILTER_ALPHA_AT_*
// mas arriba) que el extremo lejano (cerca de SETPOINT_MAX_CM) tiene una
// friccion/backlash mecanico notablemente peor que el resto de la barra, y
// necesita una respuesta MAS amortiguada (LAMBDA mas bajo), no mas agresiva.
// Se escalona con interpolate3Point() igual que los demas parametros
// dependientes de la posicion: agresivo cerca/medio (2.0, ya validado),
// suave lejos (0.8, para dar mucho mas peso relativo a la velocidad dentro
// de 's' y frenar el sobrepaso que causaba la oscilacion).
const float SMC_LAMBDA_AT_MIN = 2.0;
// MID: bajado de 2.0 a 1.55 para ganar amortiguamiento en el retorno 21->15
// (medicion serial: sobrepaso ~6cm y settle 10s). Menos peso al error = mas
// peso relativo a la velocidad dentro de s, frena mejor el sobrepaso.
const float SMC_LAMBDA_AT_MID = 1.70;  // settle_final2: mejor retorno al centro
// SMC_LAMBDA_AT_MAX: bajado de 0.8 a 0.6. Con 0.8 el salto extremo 13->23cm
// ya asentaba (dejo de oscilar sin fin), pero quedaba justo en el limite de
// 6s (6.3s medido), con una oscilacion residual lenta (~1s de periodo,
// +-0.5cm) cerca de 23cm antes de asentarse del todo. Mas amortiguamiento
// (LAMBDA mas bajo) puntualmente ahi, sin afectar la fase de acercamiento
// (que ya llegaba rapido, ~2.7-3s, y esta dominada mayormente por el termino
// de velocidad, no por este ajuste fino cerca del objetivo).
// SMC_LAMBDA_AT_MAX: 1.0 (antes 0.6). Con 0.6 el tramo final a 23cm se quedaba
// ~0.7-1.3cm corto; 1.0 da mas autoridad de posicion sin volver a 2.0 (oscilaba).
const float SMC_LAMBDA_AT_MAX = 1.10;
const float SMC_PHI = 5.0;
const float SMC_MAX_DEG = 40.0;
const float SMC_KI = 5.0;
const float SMC_INTEGRAL_LIMIT_AT_MIN = 15.0;
const float SMC_INTEGRAL_LIMIT_AT_MID = 15.0;
const float SMC_INTEGRAL_LIMIT_AT_MAX = 22.0;
// VELOCITY_FILTER_ALPHA_AT_*: filtro EMA sobre la velocidad estimada
// (diferencia de la distancia ya filtrada por Kalman entre ciclos, dividida
// por el periodo). Analogo al filtro que ya se usaba sobre el termino D del
// PID anterior, por la misma razon: diferenciar amplifica mucho el ruido
// residual del sensor. Y, tambien igual que con D_FILTER_ALPHA_AT_MIN/MID/MAX
// (ver mas arriba), el sensor Sharp GP2Y0A21 tiene mas ruido relativo lejos
// del sensor que cerca: un solo valor fijo dejaba a 25cm con una estimacion
// de velocidad mas ruidosa, empeorando el error de asentamiento ahi
// (confirmado en hardware: 25cm quedaba en ~0.7cm de error residual mientras
// 13/15cm bajaban de 0.4cm). Se escalona igual que antes, interpolando por
// SETPOINT_MIN/MID/MAX_CM via interpolate3Point().
const float VELOCITY_FILTER_ALPHA_AT_MIN = 0.6;   // menos filtrado, cerca del sensor (menos ruido relativo)
const float VELOCITY_FILTER_ALPHA_AT_MID = 0.45;  // un poco menos filtrado: frena antes el sobrepaso al centro
const float VELOCITY_FILTER_ALPHA_AT_MAX = 0.15;  // mas filtrado, lejos del sensor (mas ruido relativo)

// Compensacion de friccion/backlash CONTINUA (no un "golpe" puntual como
// applyUnstickKick(), que solo actua tras 3s de espera): un empuje constante
// y MODERADO en la direccion en la que YA se esta corrigiendo, activo en
// TODO ciclo donde la pelota esta casi detenida (velocidad baja) pero el
// error todavia es significativo. La idea es prevenir que la pelota quede
// "flotando" justo al borde de una zona muerta mecanica en primer lugar, en
// vez de esperar a que se atasque por varios segundos para recien reaccionar.
// Friccion programada: mas empuje solo lejos del sensor (meseta ~21-23cm).
const float FRICTION_COMP_DEG_AT_MIN = 11.0;
const float FRICTION_COMP_DEG_AT_MID = 14.0;
const float FRICTION_COMP_DEG_AT_MAX = 20.0;
const float FRICTION_COMP_VEL_THRESHOLD  = 0.65;
const unsigned long FRICTION_COMP_MIN_STILL_MS = 320;
const float FRICTION_COMP_ERROR_THRESHOLD = 0.55;
///////////////////////////////////////////////////////////////////

/////////////////Ajustes de comportamiento del control///////////////
const float DEADBAND = 0.7;          // Zona muerta (+-mm) alrededor del setpoint para evitar temblores.
                                      // Ahora que el termino D usa "real_error" (sin salto artificial al
                                      // cruzar el borde, ver PID_d en loop()), un valor mas amplio que 0.5
                                      // reduce la frecuencia de cruces del borde sin reintroducir el
                                      // problema del "golpe de derivada".
const float ANTIWINDUP_RANGE = 8.0;  // Rango (+-mm) donde se permite que la integral acumule

// SERVO_OUT_MIN/MAX/OFFSET + el rango de pulso de myservo.attach() (ver setup())
// definen juntos el angulo REAL que se le puede pedir al servo. Un servo
// muerto (potenciometro interno con el tope roto, girando solo hacia un
// lado) es un sintoma clasico de haberle pedido, de forma sostenida y
// repetida, un angulo mas alla de su tope mecanico real: el motor empuja con
// torque completo contra un tope fisico que no puede ceder. Los valores
// [20,160]+15=[35,175] usados originalmente, combinados con el rango de
// pulso (500,2400) de attach() (ver setup(), tambien corregido), llegaban
// MUY cerca de los extremos absolutos 0/180 en pulso (500/2400us), un rango
// que la mayoria de servos analogicos hobby no soporta de forma segura de
// forma sostenida. Se angosto el rango a [25,155]+15=[40,170] (deja 10
// grados de margen real a cada lado antes de 0/180) como capa EXTRA de
// seguridad, ademas de la correccion principal en el rango de pulso.
const int SERVO_OUT_MIN = 25;        // Limite inferior del angulo de salida del PID
const int SERVO_OUT_MAX = 155;       // Limite superior del angulo de salida del PID
const int SERVO_OFFSET  = 15;        // Compensacion mecanica para centrar el servo en ~90 grados

// SERVO_MAX_SLEW_DEG_PER_S: limite de velocidad (grados/segundo) aplicado al
// ULTIMO paso antes de escribir al servo (ver writeServoLimited() mas abajo).
// Sin esto, un cambio brusco del angulo objetivo (tipico del auto-tune, que
// puede pedir un salto de +h a -h de un ciclo a otro, ej. 130 a 50 grados de
// golpe) se traduce en un solo comando PWM pidiendole al motor el maximo
// torque instantaneo para "teletransportarse" a la nueva posicion. Limitar
// la velocidad de cambio en software fuerza una transicion mas gradual
// (similar a como se mueve el servo fisicamente de todos modos, pero
// evitando pedirle explicitamente un salto instantaneo), reduciendo el
// estres mecanico repetido sobre el engranaje/potenciometro. 400 grados/s
// sigue siendo mucho mas rapido que la dinamica de la pelota, asi que no
// afecta la calidad del control ni del auto-tune de forma perceptible.
const float SERVO_MAX_SLEW_DEG_PER_S = 400.0;
float servoWriteAngleLast = 125.0;  // Ultimo angulo realmente escrito al servo (para el limitador de velocidad). Debe coincidir con el write(125) inicial de setup().

// Escribe al servo limitando cuantos grados puede cambiar por ciclo (ver
// SERVO_MAX_SLEW_DEG_PER_S). Es el UNICO lugar del codigo que debe llamar a
// myservo.write() fuera de la posicion inicial en setup(): todas las ramas
// de loop() (MODE_AUTO, MODE_MANUAL, MODE_AUTOTUNE) deben pasar por aca.
void writeServoLimited(float targetAngle) {
  float maxStep = SERVO_MAX_SLEW_DEG_PER_S * (period / 1000.0);
  float diff = constrain(targetAngle - servoWriteAngleLast, -maxStep, maxStep);
  servoWriteAngleLast += diff;
  myservo.write(servoWriteAngleLast);
}

// PID_I_DEADBAND_DECAY: mientras el error esta dentro de la zona muerta, PID_i ya
// no queda congelado: se multiplica por este factor cada ciclo (50ms) para que
// decaiga gradualmente hacia 0 en vez de quedar "guardado" indefinidamente y
// aplicarse de golpe cuando la pelota vuelve a salir de la zona muerta. Con 0.98
// la constante de tiempo del decaimiento es de un par de segundos, lo que deja
// que compensaciones recientes de desnivel se mantengan brevemente sin acumularse
// para siempre. Mas cerca de 1.0 => decae mas lento (retiene el bias mas tiempo).
// Mas cerca de 0.0 => decae mas rapido y "olvida" antes el bias acumulado.
const float PID_I_DEADBAND_DECAY = 0.98;
///////////////////////////////////////////////////////////////////

/////////////////////////Dither anti-stiction del servo///////////////
// DITHER_AMPLITUDE_DEG: amplitud (en grados) de una pequena oscilacion que se
// suma al angulo final escrito al servo, para evitar que el mecanismo se
// "trabe" por friccion estatica (stiction) durante los periodos de inactividad
// (zona muerta). Debe ser lo bastante chica para no mover la barra de forma
// perceptible por el sensor/PID, pero lo bastante grande para sacudir el
// mecanismo. Este valor es empirico: puede necesitar ajuste en el hardware
// real; rango tipico sugerido: 0.3 a 1.5 grados.
const float DITHER_AMPLITUDE_DEG = 0.6;
const unsigned long DITHER_PERIOD_MS = 100;  // Periodo de la oscilacion del dither (ms)

// Suma una pequena oscilacion (onda cuadrada basada en millis()) al angulo
// base, solo para lo que efectivamente se escribe al servo. No debe usarse
// para alimentar el calculo del error/PID ni para actualizar servo_angle_actual.
float applyDither(float baseAngle) {
  bool phaseHigh = (millis() % DITHER_PERIOD_MS) < (DITHER_PERIOD_MS / 2);
  float offset = phaseHigh ? DITHER_AMPLITUDE_DEG : -DITHER_AMPLITUDE_DEG;
  // baseAngle ya incluye SERVO_OFFSET (viene de servo_angle_actual), asi que los
  // limites fisicos reales del servo son SERVO_OUT_MIN/MAX desplazados por el offset.
  return constrain(baseAngle + offset, SERVO_OUT_MIN + SERVO_OFFSET, SERVO_OUT_MAX + SERVO_OFFSET);
}
///////////////////////////////////////////////////////////////////

/////////////////"Golpe" anti-atasco durante MODE_AUTO////////////////
// MOTIVACION (confirmada por prueba manual en hardware real, moviendo el
// slider de angulo a distinta velocidad): el enlace mecanico servo-barra de
// este hardware en particular tiene fricción estatica/juego (stick-slip) tan
// importante que un cambio LENTO Y SOSTENIDO de angulo (que es exactamente
// lo que hace el PID en operacion normal, sobre todo cerca de la zona muerta
// donde el error cambia poco) puede no alcanzar a mover la pelota EN
// ABSOLUTO, aunque el angulo comandado sea grande. En cambio, un cambio
// RAPIDO Y BREVE (como el "golpe" del auto-tune, ver AUTOTUNE_KICK_*) si
// logra destrabarla. Esto es analogo a lo que ya soluciona el dither para la
// friccion interna del SERVO, pero a otra escala: aca el atasco es entre la
// PELOTA y el riel/varilla, no dentro del servo, y necesita mucho mas
// amplitud que el dither (0.6 grados) para superarse.
//
// MECANISMO: si el error real esta fuera de la zona muerta (la pelota
// todavia no llego) Y la pelota no se desplazo al menos
// UNSTICK_MOVEMENT_THRESHOLD_CM en los ultimos UNSTICK_DETECT_MS, se asume
// que esta atascada y se le superpone un "golpe" breve (UNSTICK_KICK_DEG
// durante UNSTICK_KICK_DURATION_MS) en la MISMA direccion en la que el PID
// ya esta empujando, antes de volver al angulo normal calculado por el PID.
// Un cooldown evita relanzar el golpe de forma continua.
// UNSTICK_MOVEMENT_THRESHOLD_CM: en pruebas reales a ~26cm (lejos del sensor),
// el ruido residual del sensor/Kalman por si solo ya genera oscilaciones de
// hasta ~1cm en pocos ciclos, incluso con la pelota realmente detenida. Un
// umbral de 0.3cm quedaba por DEBAJO de ese ruido, asi que la condicion "no
// se movio" practicamente nunca se cumplia (el ruido se interpretaba como
// movimiento real) y el golpe nunca se disparaba. Se subio bien por encima
// del ruido observado para no confundir una pelota atascada con ruido de
// sensor.
const float UNSTICK_MOVEMENT_THRESHOLD_CM = 1.5;   // desplazamiento minimo (cm) para considerar que la pelota "se esta moviendo" durante la ventana de deteccion
// UNSTICK_DETECT_MS: se probo con 1200ms y trajo un problema mas serio que el
// que resuelve. Telemetria real mostro dos escenarios con el MISMO error
// (~1.6-1.8cm) en el instante de evaluar: (1) la pelota realmente atascada
// ~9s en 16.6-16.8cm mientras el windup del integral subia el angulo de
// 129 a 146 grados hasta liberarla SOLA (sin golpe, porque el error nunca
// supero el umbral de entonces); y (2) la pelota en pleno reacomodo normal
// cerca del setpoint, donde un golpe disparado con ese mismo error la mando
// de 15.4cm a 17.7cm en vez de dejarla asentar. Es decir: el error solo NO
// distingue "atascada" de "asentando"; lo que si distingue es el TIEMPO. Se
// alarga la ventana bastante por encima de lo que tarda una correccion
// normal (que en pruebas reales resuelve en 1-3s) para que el golpe solo
// actue en estancamientos genuinamente largos, dandole antes una oportunidad
// al windup natural (que ya demostro poder resolverlo solo).
// UNSTICK_DETECT_MS: bajado de 3000 a 1800ms. En 23cm el error cruzaba
// intermitentemente por debajo de 1.0cm y reiniciaba la ventana de 3s, asi
// que el golpe NUNCA se disparaba en 12s de estancamiento medido por serial.
const unsigned long UNSTICK_DETECT_MS      = 1800;  // ventana (ms) sin ese desplazamiento minimo antes de considerar "atascada"
// UNSTICK_ERROR_THRESHOLD_CM: bajado de 1.0 a 0.75cm para no perder el conteo
// cuando el error residual en mesetas lejanas oscila ~0.9-1.4cm.
const float UNSTICK_ERROR_THRESHOLD_CM    = 0.70;
const float UNSTICK_KICK_DEG               = 35.0;  // amplitud extra (grados) del golpe, superpuesta al angulo normal del PID
// UNSTICK_KICK_DURATION_MS: se redujo de 250 a 180ms para dar un nudge mas
// corto/controlado y reducir el riesgo de sobre-impulsar la pelota mas alla
// del setpoint (visto en telemetria real con la duracion anterior).
const unsigned long UNSTICK_KICK_DURATION_MS = 180; // duracion del golpe
const unsigned long UNSTICK_COOLDOWN_MS      = 900; // tiempo minimo entre golpes consecutivos

bool unstickKickActive             = false;
unsigned long unstickKickStartMs   = 0;
unsigned long unstickStillSinceMs  = 0;   // 0 = ventana de deteccion no iniciada
float unstickRefDistance           = 0.0; // distancia al inicio de la ventana de deteccion actual
unsigned long unstickCooldownUntilMs = 0;

// Debe llamarse una vez por ciclo en MODE_AUTO, ANTES de escribir al servo,
// con el angulo ya calculado por el PID (que incluye SERVO_OFFSET). Devuelve
// el angulo a escribir, con el golpe superpuesto si corresponde. Actualiza
// el estado interno de deteccion de atasco.
float applyUnstickKick(float pidAngle) {
  // OJO: se usa UNSTICK_ERROR_THRESHOLD_CM (bastante mayor que DEADBAND), no
  // DEADBAND directamente. Con DEADBAND (0.7cm) el golpe se disparaba incluso
  // con la pelota ya cerca del setpoint (ej. error=0.8cm, apenas fuera de la
  // zona muerta), empujandola MAS LEJOS del objetivo en vez de solo rescatarla
  // cuando esta genuinamente atascada lejos de el; esto alimentaba una
  // oscilacion sostenida en vez de ayudar a asentarse. Confirmado en hardware
  // real (telemetria con "unstick_active":true justo antes de que la pelota
  // se alejara de 15.4cm a 17.7cm en vez de asentarse).
  bool errorSignificant = (real_error > UNSTICK_ERROR_THRESHOLD_CM || real_error < -UNSTICK_ERROR_THRESHOLD_CM);

  if (unstickKickActive) {
    if (millis() - unstickKickStartMs >= UNSTICK_KICK_DURATION_MS) {
      unstickKickActive = false;
      unstickCooldownUntilMs = millis() + UNSTICK_COOLDOWN_MS;
      unstickStillSinceMs = 0;
    }
  } else if (!errorSignificant) {
    unstickStillSinceMs = 0;  // pelota ya suficientemente cerca del setpoint: no aplica deteccion de atasco
  } else {
    if (unstickStillSinceMs == 0) {
      unstickStillSinceMs = millis();
      unstickRefDistance = distance;
    } else if (millis() - unstickStillSinceMs > UNSTICK_DETECT_MS) {
      bool moved = fabs(distance - unstickRefDistance) >= UNSTICK_MOVEMENT_THRESHOLD_CM;
      if (!moved && millis() > unstickCooldownUntilMs) {
        unstickKickActive   = true;
        unstickKickStartMs  = millis();
      }
      // Se reinicia la ventana en cualquier caso: si se movio, para medir el
      // siguiente tramo; si se disparo el golpe, para no evaluarlo de nuevo
      // hasta que termine (unstickKickActive ya lo cubre en el siguiente ciclo).
      unstickStillSinceMs = millis();
      unstickRefDistance  = distance;
    }
  }

  if (!unstickKickActive) return pidAngle;

  // Empuja mas fuerte en la MISMA direccion en la que el PID ya esta
  // inclinando la barra (respecto al centro real 90+SERVO_OFFSET), nunca en
  // la direccion opuesta: el golpe debe ayudar a terminar el movimiento que
  // el PID ya eligio, no pelear contra el.
  float center = 90.0 + SERVO_OFFSET;
  float kickSign = (pidAngle >= center) ? 1.0 : -1.0;
  return constrain(pidAngle + kickSign * UNSTICK_KICK_DEG, SERVO_OUT_MIN + SERVO_OFFSET, SERVO_OUT_MAX + SERVO_OFFSET);
}
///////////////////////////////////////////////////////////////////

//////////////////Filtro de Kalman (suavizado del sensor)/////////////
// Etapa adicional de filtrado ANTES del calculo del error del PID.
// get_dist(100) ya promedia 100 lecturas ADC crudas (reduce ruido de alta
// frecuencia), pero el sensor Sharp GP2Y0A21 sigue entregando una lectura
// linealizada con ruido residual notable, sobre todo porque la formula de
// linealizacion (potencia con exponente negativo) amplifica pequenas
// variaciones de voltaje en distancias cortas. El Kalman 1D suaviza esa
// senal ya promediada sin agregar el retraso excesivo de un filtro pasabajos
// fijo, porque ajusta su "confianza" (ganancia k_gain) en cada muestra.
float kalman_estimate = 15.0;   // Estimacion inicial (~igual al setpoint por defecto)
float kalman_error_cov = 1.0;   // Covarianza inicial del error de estimacion

// KALMAN_Q: varianza del RUIDO DE PROCESO, es decir, cuanto puede cambiar la
// posicion real de la pelota entre una muestra y la siguiente (cada 50ms).
// La pelota es un cuerpo fisico con inercia, pero get_dist() ya promedia 100
// lecturas ADC crudas (reduce bastante el ruido de alta frecuencia antes de
// llegar aqui), asi que el Kalman no necesita ser tan conservador: si Q es
// muy chico (ej. 0.02) la ganancia de estado estable queda muy baja (~0.16)
// y el filtro introduce un retraso notable que desfasa al termino derivativo
// del PID (sintonizado para la senal ya promediada, no para una doblemente
// suavizada). Subir Q => el filtro confia mas en la medicion nueva y responde
// mas rapido a cambios reales (menos retraso), pero deja pasar mas ruido.
const float KALMAN_Q = 0.2;

// KALMAN_R: varianza del RUIDO DE MEDICION, es decir, que tan ruidosa es la
// lectura del sensor Sharp ya promediada. Junto con KALMAN_Q de arriba, estos
// valores dan una ganancia de Kalman en estado estable k_gain = P/(P+R) ~ 0.5
// (con P la covarianza de error en estado estable), un buen equilibrio entre
// suavizado y respuesta rapida para no atrasar al termino derivativo del PID.
// Subir R => el filtro confia menos en la medicion y suaviza mas (pero con
// mas retraso). Bajar R => confia mas en la medicion, responde mas rapido,
// pero deja pasar mas ruido hacia el PID.
const float KALMAN_R = 0.4;

// Filtro de Kalman escalar (1 variable de estado: la distancia).
// Se llama una vez por iteracion del lazo, con la lectura ya promediada de
// get_dist(), y devuelve la distancia filtrada que usaran el PID, la
// telemetria y la grafica.
float kalmanFilter(float measurement) {
  // Prediccion: la incertidumbre crece con el tiempo (nadie sabe cuanto se
  // movio la pelota desde la ultima muestra) segun el ruido de proceso.
  kalman_error_cov += KALMAN_Q;

  // Actualizacion: la ganancia de Kalman decide cuanto pesa la medicion nueva
  // frente a la estimacion previa, en funcion de cuanto se confia en cada una.
  float k_gain = kalman_error_cov / (kalman_error_cov + KALMAN_R);
  kalman_estimate += k_gain * (measurement - kalman_estimate);
  kalman_error_cov *= (1 - k_gain);

  return kalman_estimate;
}
///////////////////////////////////////////////////////////////////

////////////////////////Configuracion WiFi (Access Point)////////////
const char* AP_SSID     = "PID_BallBeam";
const char* AP_PASSWORD = "12345678";  // Minimo 8 caracteres (WPA2)
///////////////////////////////////////////////////////////////////

////////////////////Estado compartido (interfaz web)/////////////////
// 'volatile' porque estas variables se escriben desde el callback del
// WebSocket (fuera del loop principal) y se leen desde loop(). Al ser
// tipos simples de 32 bits no se requieren mutex/semaforos adicionales.
enum ControlMode { MODE_AUTO, MODE_MANUAL, MODE_AUTOTUNE };
volatile ControlMode currentMode     = MODE_AUTO;
volatile float distance_setpoint     = 15;   // Distancia objetivo "comandada" desde la web. Ya no se usa
                                              // directamente en el calculo de control (ver effective_setpoint
                                              // abajo): solo es el destino hacia el cual effective_setpoint se
                                              // desliza gradualmente.
volatile float manual_angle          = 90;   // Angulo objetivo del servo cuando currentMode == MODE_MANUAL
float servo_angle_actual             = 125;  // Ultimo angulo real escrito al servo (para telemetria)

// SETPOINT_MAX_SLEW_CM_PER_S: ya NO se usa para limitar el setpoint (ver
// comentario de "RAMPA DE SETPOINT" en loop(), donde effective_setpoint ahora
// salta directo a distance_setpoint). Se deja la constante sin borrar solo
// como referencia/documentacion.
// MOTIVO DEL CAMBIO: con 3.0cm/s, un salto de 13 a 23cm (10cm) tardaba 3.3s
// SOLO en que effective_setpoint llegara al valor pedido, antes incluso de
// que la pelota empezara a acercarse en serio; eso por si solo ya consumia
// mas de la mitad del presupuesto de 6s pedido para estabilizar. La rampa
// se penso originalmente para evitar sobresaltar al gain scheduling y al
// termino derivativo del PID anterior (ver historial), pero el controlador
// SMC actual no tiene esos problemas: la derivada se calcula sobre la
// MEDICION (no sobre el error), asi que un salto de setpoint no genera un
// "golpe" de derivada; y el gain scheduling que queda (VELOCITY_FILTER_ALPHA)
// es solo un coeficiente de filtro, sin riesgo de inestabilidad si cambia de
// golpe. Ademas, el propio SMC ya satura su salida a SMC_MAX_DEG sin importar
// que tan grande sea el error instantaneo, asi que un salto de setpoint
// instantaneo simplemente hace que el controlador aplique su correccion
// maxima desde el primer ciclo (justamente lo que se necesita para llegar
// rapido), sin ningun riesgo adicional de un comando de angulo excesivo.
const float SETPOINT_MAX_SLEW_CM_PER_S = 3.0;
float effective_setpoint = 15.0;  // Arranca en el centro de la barra
///////////////////////////////////////////////////////////////////

////////////////////////Servidor Web y WebSocket/////////////////////
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
int broadcastCounter = 0;  // Cuenta iteraciones del lazo (50ms) para limitar la telemetria a ~10Hz
///////////////////////////////////////////////////////////////////

//////////////////////Auto-tune (metodo del rele)////////////////////
// El auto-tune NO se dispara solo al cambiar el setpoint (se descarto esa idea
// por inestable y dificil de razonar). Es una accion EXPLICITA, disparable
// solo por WebSocket ('autotune_start'), eligiendo uno de los 3 puntos de
// referencia del gain scheduling: SETPOINT_MIN_CM (13cm), SETPOINT_MID_CM
// (15cm) o SETPOINT_MAX_CM (23cm). YA NO tiene botones en la interfaz (ver
// 'jumpToPreset()' en INDEX_HTML): en este sistema (backlash + friccion
// stick-slip dependiente de la pelota) el auto-tune se abortaba con
// demasiada frecuencia y se remplazo por valores fijos validados a mano en
// hardware (ver KP_AT_MIN/MID/MAX etc., aunque ya no se usan en MODE_AUTO,
// que ahora corre con el controlador de modo deslizante (SMC) mas abajo).
// El sistema arranca siempre en MODE_AUTO a 15cm (ver
// 'distance_setpoint'/'effective_setpoint' arriba); el auto-tune, si se
// invoca manualmente por WebSocket, solo se ejecuta cuando se lo pide.
//
// METODO: rele con histeresis (Astrom-Hagglund). En vez de usar kp/ki/kd, se
// comanda el servo directamente a uno de dos angulos fijos (90+h / 90-h)
// segun el signo del error respecto al punto de prueba. Esto fuerza una
// oscilacion sostenida y auto-limitada de la pelota, de la que se mide:
//   - Tu: el periodo de la oscilacion (s)
//   - a : la amplitud de la oscilacion de la distancia medida (cm)
// y con eso se calcula la ganancia ultima Ku y se aplican las formulas
// clasicas de Ziegler-Nichols (metodo de oscilacion sostenida) para obtener
// Kp/Ki/Kd, que luego se convierten a las unidades particulares que usa este
// codigo (ver derivacion completa mas abajo, antes de finishAutotune()).
//
// SEGURIDAD: el auto-tune se aborta solo si la pelota sale de un rango
// ABSOLUTO seguro de la barra/sensor (AUTOTUNE_SAFE_MIN_CM/MAX_CM, NO relativo
// al punto de prueba: el metodo del rele necesita margen real para oscilar, un
// limite relativo muy ajustado se disparaba con el primer movimiento legitimo,
// antes de completar un solo ciclo) o si se agota el tiempo maximo sin
// completar suficientes ciclos de oscilacion. Al abortar o terminar, el
// sistema siempre vuelve a MODE_AUTO (nunca se queda "trabado" en autotune).
//
// NOTA (correccion 1, tras prueba en hardware real): la primera version usaba
// AUTOTUNE_RELAY_AMPLITUDE_DEG=25 (grados) y un limite de seguridad RELATIVO
// de solo 6cm respecto al punto de prueba. Con un "empujon" tan fuerte, la
// pelota recorria varios cm en un solo semiciclo, disparando el abort de
// seguridad antes de completar una sola oscilacion real ("se cancela apenas
// detecta movimiento"). El limite de seguridad paso a ser ABSOLUTO (ligado al
// rango real de la barra/sensor, no al punto de prueba).
//
// NOTA (correccion 2, tras otra prueba en hardware real): incluso subiendo la
// amplitud sostenida a 28 grados, la pelota (de ping-pong, ~2g, muy liviana)
// se quedaba quieta y el auto-tune nunca lograba completarse. Explicacion:
// la fuerza que empuja a la pelota por la inclinacion es proporcional a su
// peso (m*g*sin(angulo)), pero la friccion estatica pelota-riel NO se reduce
// en la misma proporcion para una pelota tan liviana (hay un piso de friccion
// que no depende del peso: textura del riel, punto de apoyo, etc.). Resultado:
// una inclinacion SOSTENIDA, por mas grande que sea (dentro de rangos
// razonables), puede no alcanzar nunca a "arrancar" una pelota tan liviana
// desde el reposo.
//
// SOLUCION (parcial): en vez de solo sostener el angulo, cada vez que el rele
// cambia de lado se aplica primero un "golpe" breve mas fuerte (AUTOTUNE_KICK_*)
// para romper la friccion estatica desde el reposo, y luego se sostiene el
// angulo normal (AUTOTUNE_RELAY_AMPLITUDE_DEG, el que se usa para el calculo
// de Ku). Ademas, si la pelota nunca genera error suficiente para cruzar la
// histeresis (rele "pegado" de un lado), se fuerza el cambio de lado igual
// tras AUTOTUNE_MAX_DWELL_MS (con su propio golpe incluido).
//
// NOTA (correccion 3, analisis exhaustivo tras reportes de valores de K
// incorrectos salvo en 13cm, y de que la pelota se traba en las esquinas/
// extremos de la barra durante la oscilacion):
//
// 1) CONTAMINACION del calculo de Ku por el "golpe": la formula de Ku asume
//    una amplitud de rele CONSTANTE en todo el semiciclo (ver derivacion mas
//    abajo). Pero con golpe+sostenido, una parte de cada semiciclo se manda
//    mas fuerte que AUTOTUNE_RELAY_AMPLITUDE_DEG (el valor que usa la
//    formula). Si el periodo de oscilacion es corto, el golpe es una
//    fraccion grande del semiciclo y el Ku calculado queda sesgado; ese sesgo
//    varia segun el periodo real en cada punto (13/15/25cm), lo que explica
//    por que las K calculadas eran creibles en un punto y no en otro. FIX:
//    ahora solo se promedian para Tu/amplitud los ciclos donde NINGUN cambio
//    de lado fue forzado por dwell (o sea, ciclos 100% impulsados por el
//    movimiento natural de la pelota, sin depender del golpe de arranque).
//    Los ciclos "sucios" (con algun cambio forzado) se descartan por completo
//    en vez de promediarse (ver autotunePeriodHasForced en runAutotuneStep()).
//
// 2) PELOTA ATASCADA EN UN EXTREMO/ESQUINA: si el punto de prueba esta muy
//    cerca de un tope fisico real de la barra, la pelota puede quedar
//    literalmente atrapada ahi (ningun golpe la sacara si es un tope
//    mecanico, no friccion). Antes, esto solo se resolvia esperando el
//    timeout completo (40s) sin ningun diagnostico util. FIX: se cuentan los
//    cambios de lado FORZADOS consecutivos (sin ningun cambio natural en
//    medio); si se superan AUTOTUNE_MAX_CONSECUTIVE_FORCED, se aborta de
//    inmediato con un mensaje especifico ("pelota atascada mecanicamente")
//    en vez de seguir intentando o de esperar el timeout generico.
//
// 3) LIMITES DE SEGURIDAD: se habian ampliado a mano a [6, 37]cm. 6cm esta
//    muy por debajo del rango donde el sensor Sharp es confiable (~11cm, ver
//    SETPOINT_MIN_CM) y 37cm excede el rango fisico visible de la barra
//    (BAR_MAX=30 en la interfaz): en ambos casos el "limite de seguridad" ya
//    no protegia nada porque el sensor devuelve numeros no confiables o la
//    pelota ya esta fuera de la barra. Se volvio a limites ligados al rango
//    real conocido del sensor/barra.
//
// 4) GOLPE DEMASIADO FUERTE: con AUTOTUNE_KICK_AMPLITUDE_DEG=90, el angulo
//    comandado (90+-90 = 0 o 180) queda muy por fuera de
//    [SERVO_OUT_MIN+OFFSET, SERVO_OUT_MAX+OFFSET]=[35,175] y el constrain()
//    lo recorta siempre al tope fisico configurado del servo (35 o 175) en
//    cada golpe: el servo "golpea contra la pared" en cada cambio de lado,
//    lo cual desgasta el mecanismo sin aportar mas fuerza real (ya estaba
//    saturado). Se bajo a un valor que sigue siendo un golpe notablemente mas
//    fuerte que el sostenido, pero sin saturar contra el limite.
const float AUTOTUNE_RELAY_AMPLITUDE_DEG = 40;   // h: semi-amplitud SOSTENIDA del rele (grados), la que se usa para calcular Ku
const float AUTOTUNE_KICK_AMPLITUDE_DEG  = 55;   // semi-amplitud del "golpe" breve al inicio de cada lado (90-55=35, toca el limite SERVO_OUT_MIN+OFFSET sin pasarse)
const unsigned long AUTOTUNE_KICK_DURATION_MS = 250; // duracion del golpe antes de bajar a la amplitud sostenida
const float AUTOTUNE_HYSTERESIS_CM       = DEADBAND; // banda muerta del rele: se reutiliza DEADBAND (ya validado en hardware para el ruido de este sensor)
const unsigned long AUTOTUNE_MAX_DURATION_MS = 40000; // tiempo maximo total de la prueba antes de abortar por timeout
const unsigned long AUTOTUNE_MAX_DWELL_MS    = 3000;  // si el rele lleva mas de esto del mismo lado sin que el error cruce la histeresis, se fuerza el cambio de lado (con su propio golpe)
const int AUTOTUNE_MAX_CONSECUTIVE_FORCED = 4;     // cambios de lado forzados (por dwell) SEGUIDOS, sin ningun cambio natural en medio, antes de abortar por "pelota atascada"
const int AUTOTUNE_SKIP_TRANSIENT_PERIODS = 2;     // ciclos NATURALES iniciales que se ignoran (arranque/transitorio, aun no es una oscilacion "limpia")
const int AUTOTUNE_MEASURE_PERIODS        = 3;     // ciclos NATURALES completos que se promedian para obtener Tu y la amplitud
// OJO: estos dos valores habian quedado en [6, 34] (los limites INSEGUROS que
// el comentario de arriba (punto 3) dice haber revertido), probablemente por
// una edicion manual posterior. Se corrigen de vuelta a los limites ligados
// al rango real conocido del sensor/barra.
const float AUTOTUNE_SAFE_MIN_CM = 11.0;  // Piso ABSOLUTO de seguridad: coincide con el limite real de precision del sensor (ver SETPOINT_MIN_CM). Por debajo de esto, se aborta.
const float AUTOTUNE_SAFE_MAX_CM = 28.0;  // Techo ABSOLUTO de seguridad: deja ~2cm de margen antes del extremo fisico visible de la barra (BAR_MAX=30 en la interfaz).
const float AUTOTUNE_MIN_AMPLITUDE_CM     = 0.05;  // amplitud minima considerada "oscilacion real" (evita dividir por ~0); si no se supera, finishAutotune() lo informa claramente en vez de un timeout generico

enum AutotunePoint { ATP_MIN, ATP_MID, ATP_MAX };
enum AutotuneState { ATS_IDLE, ATS_RUNNING, ATS_DONE, ATS_ABORTED };

// Prototipos explicitos de las funciones que reciben AutotunePoint/AutotuneState
// como parametro. SON NECESARIOS: el Arduino IDE genera automaticamente un
// prototipo para cada funcion del .ino e inserta TODOS esos prototipos juntos
// cerca del principio del archivo (antes de estos 'enum'). Sin esta linea, el
// prototipo auto-generado para esas funciones usaria 'AutotunePoint'/
// 'AutotuneState' antes de que existan, y la compilacion falla con "was not
// declared in this scope". Al declararlos aca nosotros mismos (con los enums
// ya visibles), el generador automatico detecta que ya existe un prototipo y
// no crea uno propio.
const char* autotunePointToStr(AutotunePoint p);
const char* autotuneStateToStr(AutotuneState s);
void startAutotune(AutotunePoint point);

// Flags 'volatile': los escribe el callback del WebSocket (otro contexto de
// ejecucion) y los consume loop() al principio de cada ciclo, igual que el
// resto del estado compartido de la interfaz (ver 'currentMode' arriba). Toda
// la logica real del auto-tune corre siempre dentro de loop(), nunca dentro
// del callback del WebSocket, para evitar condiciones de carrera con
// variables no protegidas (distance, PID_i, etc.).
volatile bool autotuneStartRequested   = false;
volatile AutotunePoint autotuneRequestedPoint = ATP_MID;
volatile bool autotuneCancelRequested  = false;
volatile bool autotuneApplyRequested   = false;

// 'autotuneState' tambien se LEE desde el callback del WebSocket (para
// ignorar comandos normales de modo/setpoint mientras la prueba esta en
// curso), por eso es volatile, igual que 'currentMode'.
volatile AutotuneState autotuneState = ATS_IDLE;
AutotunePoint autotunePoint          = ATP_MID;
float autotuneTargetCm               = SETPOINT_MID_CM;
const char* autotuneMessage          = "Inactivo.";  // Solo se le asignan literales de texto fijos, nunca texto armado en tiempo de ejecucion

// --- Estado interno del rele (solo se usa/actualiza dentro de loop(), via runAutotuneStep()) ---
bool autotuneCommandHigh      = false; // true: comanda angulo alto (90+h); false: angulo bajo (90-h)
bool autotuneHaveLastRising   = false; // true una vez que se detecto el primer flanco de subida del rele
unsigned long autotuneLastRisingMs = 0;
unsigned long autotuneSideStartMs  = 0; // momento en que empezo el lado actual del rele (natural o forzado por dwell); controla la fase de "golpe" (ver AUTOTUNE_KICK_*)
unsigned long autotuneStartMs      = 0;
float autotuneCycleMax = 0, autotuneCycleMin = 0;  // max/min de 'distance' durante el ciclo de oscilacion actual
int autotuneCyclesSeen = 0;            // ciclos NATURALES vistos (incluye los del transitorio, que se descartan). Los ciclos con algun cambio forzado NO se cuentan aca (ver autotunePeriodHasForced).
int autotuneMeasuredCycles = 0;        // ciclos NATURALES ya promediados (fuera del transitorio)
float autotunePeriodSumMs   = 0;       // suma de periodos NATURALES medidos (ms), para promediar
float autotuneAmplitudeSumCm = 0;      // suma de amplitudes NATURALES medidas (cm), para promediar
bool autotunePeriodHasForced = false;  // true si el ciclo actualmente "abierto" (desde el ultimo flanco de subida) tuvo algun cambio de lado forzado por dwell: se descarta por completo al cerrarse, no se promedia (ver NOTA "correccion 3")
int autotuneConsecutiveForced = 0;     // cambios de lado forzados SEGUIDOS sin ningun cambio natural en medio; si supera AUTOTUNE_MAX_CONSECUTIVE_FORCED, se aborta por "pelota atascada"

// Resultado de la ultima corrida exitosa (en las unidades propias de este codigo, listas para KP/KI/KD_AT_*)
float autotuneResultKp = 0, autotuneResultKi = 0, autotuneResultKd = 0;

const char* autotunePointToStr(AutotunePoint p) {
  switch (p) {
    case ATP_MIN: return "min";
    case ATP_MAX: return "max";
    default:      return "mid";
  }
}

const char* autotuneStateToStr(AutotuneState s) {
  switch (s) {
    case ATS_RUNNING:  return "running";
    case ATS_DONE:     return "done";
    case ATS_ABORTED:  return "aborted";
    default:           return "idle";
  }
}

// Vuelve a un estado de control seguro y conocido tras terminar/abortar el
// auto-tune: MODE_AUTO, con la Integral y el filtro de la derivada limpios
// (para no heredar un bias o un pico residual de la oscilacion forzada del rele).
void returnToAutoAfterAutotune() {
  currentMode = MODE_AUTO;
  PID_i = 0;
  PID_d_filtered = 0;
  smc_integral = 0;
  ball_velocity_filtered = 0;
  frictionCompStillSinceMs = 0;
}

void abortAutotune(const char *reason) {
  autotuneState = ATS_ABORTED;
  autotuneMessage = reason;
  returnToAutoAfterAutotune();
}

// Inicia una nueva corrida de auto-tune en uno de los 3 puntos de referencia.
// Fuerza distance_setpoint/effective_setpoint al punto elegido (sin rampa,
// de golpe) para que la prueba arranque ya centrada en el objetivo correcto.
void startAutotune(AutotunePoint point) {
  autotunePoint = point;
  switch (point) {
    case ATP_MIN: autotuneTargetCm = SETPOINT_MIN_CM; break;
    case ATP_MAX: autotuneTargetCm = SETPOINT_MAX_CM; break;
    default:      autotuneTargetCm = SETPOINT_MID_CM; break;
  }
  distance_setpoint   = autotuneTargetCm;
  effective_setpoint  = autotuneTargetCm;
  PID_i = 0;
  PID_d_filtered = 0;
  smc_integral = 0;
  ball_velocity_filtered = 0;
  frictionCompStillSinceMs = 0;

  float initialError = autotuneTargetCm - distance;
  autotuneCommandHigh   = (initialError < 0);  // si ya esta "lejos" (error negativo), arranca acercando (angulo alto)
  autotuneHaveLastRising = false;
  autotuneCycleMax = distance;
  autotuneCycleMin = distance;
  autotuneCyclesSeen     = 0;
  autotuneMeasuredCycles = 0;
  autotunePeriodSumMs    = 0;
  autotuneAmplitudeSumCm = 0;
  autotunePeriodHasForced   = false;
  autotuneConsecutiveForced = 0;
  autotuneStartMs = millis();
  autotuneSideStartMs = autotuneStartMs;  // el primer lado tambien arranca con su fase de "golpe" (ver AUTOTUNE_KICK_*)

  autotuneState   = ATS_RUNNING;
  autotuneMessage = "Auto-tune en progreso...";
  currentMode = MODE_AUTOTUNE;
}

// --- Derivacion de las formulas usadas en finishAutotune() ---
// Durante el rele, servo_angle_actual se comanda directo a 90+h o 90-h (sin
// pasar por kp/ki/kd ni por el map() del modo automatico). Pero en MODE_AUTO,
// servo_angle_actual = map(PID_total,-150,150,150,0) + SERVO_OFFSET, que es
// equivalente a "servo_angle_actual = 90 - 0.5*PID_total" (sin saturar). Por
// lo tanto, comandar 90+h equivale a un PID_total de -2h, y comandar 90-h
// equivale a un PID_total de +2h: el rele fuerza PID_total a un valor de
// +-2h. Es decir, la semi-amplitud del rele EN LAS MISMAS UNIDADES que
// PID_total (y por lo tanto que "kp*error") es d = 2h.
//
// Formula clasica del rele (Astrom-Hagglund): Ku = 4*d / (pi*a), con d la
// semi-amplitud del rele y "a" la semi-amplitud (pico a pico /2) de la
// oscilacion de la variable controlada (aca, 'distance' en cm). Sustituyendo
// d=2h: Ku = 8*h / (pi*a). Esta Ku ya esta en las mismas unidades que "kp"
// (PID_total = kp*error), no requiere mas conversion para el termino P.
//
// Ziegler-Nichols (oscilacion sostenida) tiene VARIAS tablas de formulas segun
// que tan agresivo se quiera el resultado. La tabla "PID clasica"
// (Kp=0.6*Ku, Ti=Tu/2, Td=Tu/8) es la mas conocida, pero es
// intencionalmente agresiva: prioriza rechazar perturbaciones rapido por
// sobre la estabilidad, y en la practica suele dar ~25% de sobre-oscilacion
// (overshoot) incluso en el caso "de libro". Para un sistema tan delicado
// como este (pelota+barra, con margen de error de solo unos cm antes de
// caerse), esa tabla resulto demasiado agresiva en hardware real: el sistema
// "funciona" pero no llega a asentarse, oscila y se termina volcando. Por eso
// se usa la tabla "sin sobre-oscilacion" (no overshoot) de Ziegler-Nichols,
// pensada exactamente para evitar ese problema:
//   Kp = 0.2*Ku ; Ti = Tu/2 (=> Ki_continua = 2*Kp/Tu) ; Td = Tu/3 (=> Kd_continua = Kp*Tu/3)
// (vs. la tabla clasica, esto es ~3 veces menos Kp/Ki, y proporcionalmente MAS
// Td relativo a Kp, lo que ayuda a frenar el overshoot en vez de producirlo).
// Esas Ki_continua/Kd_continua estan en la convencion de libro de texto
// (salida = Kp*e + Ki_continua*integral(e dt) + Kd_continua*de/dt, con dt en
// SEGUNDOS). Este codigo, en cambio, acumula PID_i sumando ki*error cada
// ciclo SIN dividir por dt (equivale a ki_propio = Ki_continua * dt_segundos,
// con dt_segundos = period/1000), y calcula PID_d como
// -kd*(distancia-distancia_previa)/period (con period en MILISEGUNDOS, no
// segundos; equivale a kd_propio = Kd_continua * 1000). Ver comentarios en
// loop() para el detalle de PID_i/PID_d.
void finishAutotune() {
  float TuMs = autotunePeriodSumMs / autotuneMeasuredCycles;
  float aCm  = autotuneAmplitudeSumCm / autotuneMeasuredCycles;

  if (aCm < AUTOTUNE_MIN_AMPLITUDE_CM) {
    abortAutotune("Oscilacion demasiado pequena para calcular ganancias confiables.");
    return;
  }

  float TuS = TuMs / 1000.0;
  float h   = AUTOTUNE_RELAY_AMPLITUDE_DEG;
  float Ku  = (8.0 * h) / (PI * aCm);

  float kpZN    = 0.2 * Ku;
  float KiCont  = (2.0 * kpZN) / TuS;
  float KdCont  = (kpZN * TuS) / 3.0;

  autotuneResultKp = kpZN;
  autotuneResultKi = KiCont * (period / 1000.0);
  autotuneResultKd = KdCont * 1000.0;

  autotuneState   = ATS_DONE;
  autotuneMessage = "Auto-tune completado. Revisa los valores antes de aplicarlos.";
  returnToAutoAfterAutotune();
}

// Escribe el resultado de la ultima corrida exitosa en las constantes de gain
// scheduling correspondientes al punto probado. Cambios en RAM unicamente: se
// pierden si el ESP32 se reinicia (no hay persistencia en flash todavia).
void applyAutotuneResult() {
  if (autotuneState != ATS_DONE) return;
  switch (autotunePoint) {
    case ATP_MIN:
      KP_AT_MIN = autotuneResultKp; KI_AT_MIN = autotuneResultKi; KD_AT_MIN = autotuneResultKd;
      break;
    case ATP_MAX:
      KP_AT_MAX = autotuneResultKp; KI_AT_MAX = autotuneResultKi; KD_AT_MAX = autotuneResultKd;
      break;
    default:
      KP_AT_MID = autotuneResultKp; KI_AT_MID = autotuneResultKi; KD_AT_MID = autotuneResultKd;
      break;
  }
  autotuneMessage = "Valores aplicados (en RAM; se pierden al reiniciar el ESP32).";
  autotuneState = ATS_IDLE;
}

// Un paso (un ciclo de loop(), cada 'period' ms) del control por rele. Usa
// 'real_error' (ya calculado al principio de loop() como effective_setpoint -
// distance) como el error de la prueba, valido porque durante el auto-tune
// effective_setpoint se mantiene fijo en autotuneTargetCm (ver startAutotune()).
void runAutotuneStep() {
  bool wantHigh = autotuneCommandHigh;
  if (real_error > AUTOTUNE_HYSTERESIS_CM) wantHigh = false;       // pelota muy cerca del objetivo/sensor -> alejar (angulo bajo)
  else if (real_error < -AUTOTUNE_HYSTERESIS_CM) wantHigh = true;  // pelota muy lejos del objetivo -> acercar (angulo alto)

  // Si la pelota no responde (no genera suficiente error para cruzar la
  // histeresis) y el rele lleva demasiado tiempo del mismo lado, forzamos el
  // cambio de lado igual: sin esto, una pelota que no se mueve deja al rele
  // "pegado" de un lado para siempre y el auto-tune nunca detecta ni un ciclo
  // (ver NOTA "correccion 2" mas arriba).
  bool forcedByDwell = (millis() - autotuneSideStartMs > AUTOTUNE_MAX_DWELL_MS);
  if (forcedByDwell) wantHigh = !autotuneCommandHigh;

  if (wantHigh != autotuneCommandHigh) {
    autotuneCommandHigh = wantHigh;
    autotuneSideStartMs = millis();  // arranca la fase de "golpe" (ver AUTOTUNE_KICK_*) para este nuevo lado

    if (forcedByDwell) {
      // Este cambio de lado NO fue causado por el movimiento real de la
      // pelota: contamina el ciclo actualmente abierto (se descartara al
      // cerrarse, ver mas abajo) y cuenta para la deteccion de "atascada".
      autotunePeriodHasForced = true;
      autotuneConsecutiveForced++;
      if (autotuneConsecutiveForced >= AUTOTUNE_MAX_CONSECUTIVE_FORCED) {
        abortAutotune("Abortado: la pelota parece atascada mecanicamente (no responde ni al golpe de arranque). Revisa el riel/la pelota en ese punto de la barra.");
        return;
      }
    } else {
      autotuneConsecutiveForced = 0;  // hubo un cambio natural: la pelota SI esta respondiendo, se reinicia el contador de "atascada"
    }

    if (wantHigh) {
      // Flanco de subida: cierra el periodo anterior (si ya habia uno) y abre uno nuevo.
      unsigned long now = millis();
      if (autotuneHaveLastRising) {
        unsigned long periodMs  = now - autotuneLastRisingMs;
        float amplitudeCm       = (autotuneCycleMax - autotuneCycleMin) / 2.0;
        // Solo se promedian los ciclos 100% NATURALES (ver NOTA "correccion 3"
        // mas arriba): un ciclo con algun cambio forzado por dwell no refleja
        // la dinamica real de la pelota y sesgaria el calculo de Ku/Tu, asi
        // que se descarta por completo (ni transitorio ni medido).
        if (!autotunePeriodHasForced) {
          autotuneCyclesSeen++;
          if (autotuneCyclesSeen > AUTOTUNE_SKIP_TRANSIENT_PERIODS) {
            autotunePeriodSumMs   += periodMs;
            autotuneAmplitudeSumCm += amplitudeCm;
            autotuneMeasuredCycles++;
          }
        }
      }
      autotuneLastRisingMs  = now;
      autotuneHaveLastRising = true;
      autotuneCycleMax = distance;
      autotuneCycleMin = distance;
      autotunePeriodHasForced = forcedByDwell;  // el nuevo ciclo que arranca ahora hereda "contaminado" si este mismo flanco de subida fue forzado
    }
  }
  if (distance > autotuneCycleMax) autotuneCycleMax = distance;
  if (distance < autotuneCycleMin) autotuneCycleMin = distance;

  // "Golpe" breve al inicio de cada lado (angulo mas fuerte que el sostenido)
  // para romper la friccion estatica desde el reposo; despues de
  // AUTOTUNE_KICK_DURATION_MS se baja a la amplitud sostenida normal (la que
  // usa finishAutotune() para calcular Ku, ver AUTOTUNE_RELAY_AMPLITUDE_DEG).
  bool inKickPhase = (millis() - autotuneSideStartMs) < AUTOTUNE_KICK_DURATION_MS;
  float amplitude = inKickPhase ? AUTOTUNE_KICK_AMPLITUDE_DEG : AUTOTUNE_RELAY_AMPLITUDE_DEG;
  float relayAngle = autotuneCommandHigh ? (90.0 + amplitude) : (90.0 - amplitude);
  servo_angle_actual = constrain(relayAngle, SERVO_OUT_MIN + SERVO_OFFSET, SERVO_OUT_MAX + SERVO_OFFSET);
  writeServoLimited(servo_angle_actual);  // Sin dither: no debe contaminar la medicion del rele

  if (distance < AUTOTUNE_SAFE_MIN_CM || distance > AUTOTUNE_SAFE_MAX_CM) {
    abortAutotune("Abortado por seguridad: la pelota salio del rango seguro de la barra/sensor.");
  } else if (millis() - autotuneStartMs > AUTOTUNE_MAX_DURATION_MS) {
    abortAutotune("Abortado por tiempo agotado: no se detecto una oscilacion sostenida.");
  } else if (autotuneMeasuredCycles >= AUTOTUNE_MEASURE_PERIODS) {
    finishAutotune();
  }
}
///////////////////////////////////////////////////////////////////

////////////////////////Pagina web embebida///////////////////////////
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 PID Balanza y Bola - Control Completo</title>
<style>
  :root {
    --bg: #0b1220;
    --panel: #152033;
    --panel2: #0f172a;
    --border: #243447;
    --accent: #22c55e;
    --accent-dim: #16a34a;
    --cyan: #38bdf8;
    --danger: #f87171;
    --text: #e2e8f0;
    --muted: #94a3b8;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    padding: 14px 16px 56px;
  }
  .app { max-width: 1100px; margin: 0 auto; }
  header {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 16px;
    padding-bottom: 12px;
    border-bottom: 1px solid var(--border);
  }
  .menu-icon {
    width: 28px; height: 22px; display: flex; flex-direction: column;
    justify-content: space-between; flex-shrink: 0;
  }
  .menu-icon span {
    display: block; height: 3px; border-radius: 2px;
    background: var(--accent); box-shadow: 0 0 8px rgba(34,197,94,0.55);
  }
  header h1 {
    margin: 0; font-size: 1.15rem; font-weight: 700; letter-spacing: 0.2px;
  }
  .top-row {
    display: grid;
    grid-template-columns: 1fr;
    gap: 16px;
    margin-bottom: 16px;
  }
  .panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 14px;
    padding: 16px;
  }
  .mode-switch { display: flex; gap: 8px; margin-bottom: 14px; }
  .mode-switch button, .preset-row button {
    flex: 1;
    padding: 11px 10px;
    border: 1px solid var(--border);
    border-radius: 10px;
    background: #1a2740;
    color: var(--text);
    font-size: 0.95rem;
    cursor: pointer;
    transition: background 0.15s, box-shadow 0.15s, border-color 0.15s, color 0.15s;
  }
  .mode-switch button.active, .preset-row button.active {
    background: var(--accent);
    color: #052e16;
    font-weight: 700;
    border-color: var(--accent);
    box-shadow: 0 0 16px rgba(34,197,94,0.45);
  }
  label { display: block; margin: 12px 0 6px; font-size: 0.88rem; color: var(--muted); }
  .value-row { display: flex; align-items: center; gap: 12px; }
  .value-row .val {
    min-width: 48px; text-align: right; font-weight: 700;
    color: var(--accent); font-variant-numeric: tabular-nums;
  }
  input[type=range] {
    -webkit-appearance: none; appearance: none;
    width: 100%; height: 6px; border-radius: 3px;
    background: #243447; outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; appearance: none;
    width: 20px; height: 20px; border-radius: 50%;
    background: var(--bg); border: 3px solid var(--accent);
    box-shadow: 0 0 10px rgba(34,197,94,0.5); cursor: pointer;
  }
  input[type=range]::-moz-range-thumb {
    width: 20px; height: 20px; border-radius: 50%;
    background: var(--bg); border: 3px solid var(--accent);
    box-shadow: 0 0 10px rgba(34,197,94,0.5); cursor: pointer;
  }
  fieldset { border: none; padding: 0; margin: 0; }
  fieldset:disabled { opacity: 0.4; }
  fieldset:disabled input { cursor: not-allowed; }
  .stats {
    display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 16px;
  }
  .stat {
    background: var(--panel2); border: 1px solid var(--border);
    border-radius: 10px; padding: 12px 10px; text-align: center;
  }
  .stat .label { font-size: 0.72rem; color: var(--muted); margin: 0 0 4px; }
  .stat .num {
    font-size: 1.25rem; font-weight: 700; color: var(--accent);
    font-variant-numeric: tabular-nums;
  }
  .gauge-panel {
    display: flex; flex-direction: column; align-items: center; justify-content: center;
    min-height: 280px;
  }
  .gauge-panel .panel-title {
    align-self: stretch; text-align: center; margin: 0 0 8px;
    font-size: 0.95rem; color: var(--muted); font-weight: 600;
  }
  #gaugeCanvas {
    width: 100%; max-width: 320px; height: 260px;
    display: block; background: transparent;
  }
  .bottom-row {
    display: grid;
    grid-template-columns: 1fr;
    gap: 16px;
  }
  .charts-wrap {
    display: grid;
    grid-template-columns: 1fr;
    gap: 16px;
  }
  .chart-panel label { margin-top: 0; }
  .chart-panel canvas.chart {
    width: 100%; height: 200px; display: block;
    background: var(--panel2); border-radius: 8px;
  }
  .legend {
    display: flex; gap: 14px; justify-content: center;
    margin-top: 8px; font-size: 0.78rem; color: var(--muted); flex-wrap: wrap;
  }
  .legend span { display: inline-flex; align-items: center; gap: 6px; }
  .legend .dot {
    width: 10px; height: 10px; border-radius: 50%; display: inline-block;
  }
  .presets-panel .panel-title {
    margin: 0 0 12px; font-size: 0.85rem; color: var(--muted); font-weight: 600;
  }
  .preset-row { display: flex; flex-direction: column; gap: 10px; }
  .preset-row button { width: 100%; padding: 14px; font-size: 1.05rem; }
  .conn-status {
    position: fixed; right: 16px; bottom: 14px;
    display: flex; align-items: center; gap: 8px;
    padding: 8px 12px; border-radius: 999px;
    background: rgba(15, 23, 42, 0.92);
    border: 1px solid var(--border);
    font-size: 0.85rem; font-weight: 600; color: var(--muted);
    z-index: 20;
  }
  .conn-status .bolt {
    width: 14px; height: 14px; display: inline-block;
  }
  .conn-status.online { color: var(--accent); border-color: rgba(34,197,94,0.45); }
  .conn-status.online .bolt { filter: drop-shadow(0 0 6px rgba(34,197,94,0.8)); }
  .conn-status.offline { color: var(--danger); }

  .beam-panel { margin-bottom: 16px; }
  .beam-panel .panel-title {
    margin: 0 0 10px; font-size: 0.85rem; color: var(--muted); font-weight: 600;
  }
  .beam-track {
    position: relative;
    height: 56px;
    margin: 0 8px;
    background: linear-gradient(180deg, #121824 0%, #0c1220 100%);
    border: 1px solid var(--border);
    border-radius: 12px;
    overflow: hidden;
  }
  .beam-rail {
    position: absolute;
    left: 18px; right: 18px; top: 50%;
    height: 4px;
    background: #334155;
    border-radius: 2px;
    transform: translateY(-50%);
    box-shadow: 0 0 0 1px rgba(34,197,94,0.12);
  }
  .beam-marker {
    position: absolute;
    top: 0; bottom: 0;
    transform: translateX(-50%);
    transition: left 0.12s linear;
    pointer-events: none;
  }
  .beam-setpoint {
    width: 3px;
    background: var(--accent);
    box-shadow: 0 0 10px rgba(34,197,94,0.85);
    z-index: 2;
  }
  .beam-setpoint::before {
    content: '';
    position: absolute;
    top: 4px; left: 50%;
    width: 0; height: 0;
    border-left: 6px solid transparent;
    border-right: 6px solid transparent;
    border-top: 8px solid var(--accent);
    transform: translateX(-50%);
  }
  .beam-ball {
    width: 22px; height: 22px;
    top: 50%;
    margin-top: -11px;
    background: radial-gradient(circle at 35% 30%, #7dd3fc, #0ea5e9 55%, #0369a1);
    border: 2px solid #e0f2fe;
    border-radius: 50%;
    box-shadow: 0 0 14px rgba(56,189,248,0.85);
    z-index: 3;
  }
  .beam-labels {
    display: flex; justify-content: space-between;
    margin: 8px 18px 0;
    font-size: 0.72rem; color: var(--muted);
    font-variant-numeric: tabular-nums;
  }
  .beam-legend {
    display: flex; gap: 16px; justify-content: center;
    margin-top: 10px; font-size: 0.78rem; color: var(--muted);
  }
  .beam-legend span { display: inline-flex; align-items: center; gap: 6px; }
  .beam-legend .dot-ball {
    width: 12px; height: 12px; border-radius: 50%;
    background: #38bdf8; box-shadow: 0 0 8px rgba(56,189,248,0.7);
  }
  .beam-legend .dot-sp {
    width: 3px; height: 14px; background: var(--accent);
    box-shadow: 0 0 8px rgba(34,197,94,0.7);
  }

  @media (min-width: 900px) {
    .top-row { grid-template-columns: 1.05fr 0.95fr; align-items: stretch; }
    .bottom-row { grid-template-columns: 1fr 180px; align-items: start; }
    .charts-wrap { grid-template-columns: 1fr 1fr; }
    .preset-row { height: 100%; justify-content: center; }
  }
</style>
</head>
<body>
  <div class="app">
    <header>
      <div class="menu-icon" aria-hidden="true"><span></span><span></span><span></span></div>
      <h1>ESP32 PID Balanza y Bola - Control Completo</h1>
    </header>

    <div class="top-row">
      <div class="panel controls">
        <div class="mode-switch">
          <button id="btnAuto" class="active" onclick="setMode('auto')">Automatico</button>
          <button id="btnManual" onclick="setMode('manual')">Manual</button>
        </div>

        <label for="setpointRange">Setpoint (cm)</label>
        <div class="value-row">
          <input type="range" id="setpointRange" min="13" max="21" step="0.5" value="15">
          <span class="val" id="setpointVal">15.0</span>
        </div>

        <fieldset id="manualFieldset" disabled>
          <label for="manualRange">Angulo manual</label>
          <div class="value-row">
            <input type="range" id="manualRange" min="20" max="160" step="1" value="90">
            <span class="val" id="manualVal">90</span>
          </div>
        </fieldset>

        <div class="stats">
          <div class="stat"><div class="label">Distancia actual</div><div class="num" id="statDistance">--</div></div>
          <div class="stat"><div class="label">Error</div><div class="num" id="statError">--</div></div>
          <div class="stat"><div class="label">Modo activo</div><div class="num" id="statMode">--</div></div>
          <div class="stat"><div class="label">Angulo servo</div><div class="num" id="statAngle">--</div></div>
        </div>
      </div>

      <div class="panel gauge-panel">
        <div class="panel-title">Control Manual de Servo</div>
        <canvas id="gaugeCanvas" width="320" height="260"></canvas>
      </div>
    </div>

    <div class="panel beam-panel">
      <div class="panel-title">Posicion en la barra (0-30 cm)</div>
      <div class="beam-track" id="beamTrack">
        <div class="beam-rail"></div>
        <div class="beam-marker beam-setpoint" id="barSetpoint" style="left:50%"></div>
        <div class="beam-marker beam-ball" id="barBall" style="left:50%"></div>
      </div>
      <div class="beam-labels">
        <span>0 cm (sensor)</span>
        <span>15 cm</span>
        <span>30 cm</span>
      </div>
      <div class="beam-legend">
        <span><i class="dot-ball"></i> Pelota</span>
        <span><i class="dot-sp"></i> Setpoint</span>
      </div>
    </div>

    <div class="bottom-row">
      <div class="charts-wrap">
        <div class="panel chart-panel">
          <label>Distancia Real vs Setpoint (cm)</label>
          <canvas id="chartDist" class="chart"></canvas>
          <div class="legend">
            <span><i class="dot" style="background:#38bdf8"></i> Real</span>
            <span><i class="dot" style="background:#22c55e"></i> Setpoint</span>
          </div>
        </div>
        <div class="panel chart-panel">
          <label>Error Real (cm)</label>
          <canvas id="chartError" class="chart"></canvas>
          <div class="legend">
            <span><i class="dot" style="background:#22c55e"></i> Error</span>
          </div>
        </div>
      </div>

      <div class="panel presets-panel">
        <div class="panel-title">Presets</div>
        <div class="preset-row">
          <button id="atBtnMin" onclick="jumpToPreset(13)">13 cm</button>
          <button id="atBtnMid" class="active" onclick="jumpToPreset(15)">15 cm</button>
          <button id="atBtnMax" onclick="jumpToPreset(21)">21 cm</button>
        </div>
      </div>
    </div>
  </div>

  <div id="connStatus" class="conn-status offline">
    <svg class="bolt" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
      <path d="M13 2L4 14h7l-1 8 10-14h-7l0-6z"/>
    </svg>
    <span id="connLabel">Desconectado</span>
  </div>

<script>
  const MAX_POINTS = 300;
  const GAUGE_MIN = 20, GAUGE_MAX = 160;
  const BAR_MIN = 0, BAR_MAX = 30;
  let distArr = [], spArr = [], errArr = [];
  let currentMode = 'auto';
  let gaugeAngle = 90;

  function clampPct(v) {
    const pct = ((v - BAR_MIN) / (BAR_MAX - BAR_MIN)) * 100;
    return Math.max(0, Math.min(100, pct));
  }

  function updateBeam(distance, setpoint) {
    const pad = 18;
    const track = document.getElementById('beamTrack');
    const usable = Math.max(1, track.clientWidth - pad * 2);
    const ball = document.getElementById('barBall');
    const sp = document.getElementById('barSetpoint');
    ball.dataset.dist = distance;
    ball.style.left = (pad + (clampPct(distance) / 100) * usable) + 'px';
    sp.style.left = (pad + (clampPct(setpoint) / 100) * usable) + 'px';
  }
  window.addEventListener('resize', () => {
    const ball = document.getElementById('barBall');
    if (!ball || !ball.dataset.dist) return;
    updateBeam(parseFloat(ball.dataset.dist), parseFloat(setpointRange.value));
  });

  const connStatus = document.getElementById('connStatus');
  const connLabel = document.getElementById('connLabel');
  function setConn(online) {
    connStatus.classList.toggle('online', online);
    connStatus.classList.toggle('offline', !online);
    connLabel.textContent = online ? 'Conectado' : 'Desconectado';
  }

  const ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => setConn(true);
  ws.onclose = () => setConn(false);
  ws.onerror = () => setConn(false);
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    updateUI(d);
    pushSample(d);
    drawCharts();
  };

  function sendCmd(obj) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
    }
  }

  function setMode(m) {
    currentMode = m;
    applyModeUI(m);
    sendCmd({ type: 'mode', value: m });
  }

  function applyModeUI(m) {
    document.getElementById('btnAuto').classList.toggle('active', m === 'auto');
    document.getElementById('btnManual').classList.toggle('active', m === 'manual');
    document.getElementById('manualFieldset').disabled = (m !== 'manual');
  }

  const setpointRange = document.getElementById('setpointRange');
  const setpointVal = document.getElementById('setpointVal');
  setpointRange.addEventListener('input', () => {
    const v = parseFloat(setpointRange.value);
    setpointVal.textContent = v.toFixed(1);
    highlightPreset(v);
    const ballEl = document.getElementById('barBall');
    const distGuess = ballEl && ballEl.dataset.dist ? parseFloat(ballEl.dataset.dist) : v;
    updateBeam(distGuess, v);
  });
  setpointRange.addEventListener('change', () => {
    sendCmd({ type: 'setpoint', value: parseFloat(setpointRange.value) });
    highlightPreset(parseFloat(setpointRange.value));
  });

  const manualRange = document.getElementById('manualRange');
  const manualVal = document.getElementById('manualVal');
  manualRange.addEventListener('input', () => {
    manualVal.textContent = manualRange.value;
    gaugeAngle = parseFloat(manualRange.value);
    drawGauge(gaugeAngle);
    sendCmd({ type: 'manual_angle', value: parseFloat(manualRange.value) });
  });

  function jumpToPreset(value) {
    setpointRange.value = value;
    setpointVal.textContent = value.toFixed(1);
    highlightPreset(value);
    sendCmd({ type: 'setpoint', value: value });
  }

  function highlightPreset(value) {
    const map = { 13: 'atBtnMin', 15: 'atBtnMid', 21: 'atBtnMax' };
    ['atBtnMin', 'atBtnMid', 'atBtnMax'].forEach((id) => {
      document.getElementById(id).classList.remove('active');
    });
    const key = Math.round(value);
    if (map[key] && Math.abs(value - key) < 0.05) {
      document.getElementById(map[key]).classList.add('active');
    }
  }

  const MODE_LABELS = { auto: 'Automatico', manual: 'Manual', autotune: 'Auto-tune' };

  function updateUI(d) {
    document.getElementById('statDistance').textContent = d.distance.toFixed(1) + ' cm';
    document.getElementById('statError').textContent = d.error.toFixed(1) + ' cm';
    document.getElementById('statMode').textContent = MODE_LABELS[d.mode] || d.mode;
    document.getElementById('statAngle').textContent = Math.round(d.servo_angle) + ' deg';

    gaugeAngle = d.servo_angle;
    drawGauge(gaugeAngle);
    updateBeam(d.distance, d.setpoint);

    if (typeof d.setpoint === 'number') {
      if (document.activeElement !== setpointRange) {
        setpointRange.value = d.setpoint;
        setpointVal.textContent = d.setpoint.toFixed(1);
      }
      highlightPreset(d.setpoint);
    }

    if (d.mode !== currentMode) {
      currentMode = d.mode;
      applyModeUI(currentMode);
    }
  }

  function pushSample(d) {
    distArr.push(d.distance);
    spArr.push(d.setpoint);
    errArr.push(d.error);
    if (distArr.length > MAX_POINTS) {
      distArr.shift(); spArr.shift(); errArr.shift();
    }
  }

  const SAMPLE_PERIOD_S = 0.1;

  const gaugeCanvas = document.getElementById('gaugeCanvas');
  const gaugeCtx = gaugeCanvas.getContext('2d');

  function drawGauge(angleDeg) {
    const c = gaugeCanvas;
    const ctx = gaugeCtx;
    const w = c.width, h = c.height;
    ctx.clearRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h * 0.56;
    const r = Math.min(w, h) * 0.38;
    const a = Math.max(GAUGE_MIN, Math.min(GAUGE_MAX, angleDeg));
    const t = (a - GAUGE_MIN) / (GAUGE_MAX - GAUGE_MIN);
    const start = Math.PI * 0.75;
    const end = Math.PI * 2.25;
    const needle = start + t * (end - start);

    ctx.beginPath();
    ctx.arc(cx, cy, r + 10, start, end);
    ctx.strokeStyle = '#243447';
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(cx, cy, r + 10, start, needle);
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';
    ctx.shadowColor = 'rgba(34,197,94,0.65)';
    ctx.shadowBlur = 12;
    ctx.stroke();
    ctx.shadowBlur = 0;

    const ticks = 7;
    for (let i = 0; i <= ticks; i++) {
      const ang = start + (i / ticks) * (end - start);
      const x1 = cx + Math.cos(ang) * (r - 4);
      const y1 = cy + Math.sin(ang) * (r - 4);
      const x2 = cx + Math.cos(ang) * (r + 2);
      const y2 = cy + Math.sin(ang) * (r + 2);
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.strokeStyle = '#64748b';
      ctx.lineWidth = 2;
      ctx.stroke();
      const val = GAUGE_MIN + (i / ticks) * (GAUGE_MAX - GAUGE_MIN);
      ctx.fillStyle = '#94a3b8';
      ctx.font = '11px "Segoe UI", Tahoma, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(Math.round(val)), cx + Math.cos(ang) * (r - 22), cy + Math.sin(ang) * (r - 22));
    }

    const nx = cx + Math.cos(needle) * (r - 28);
    const ny = cy + Math.sin(needle) * (r - 28);
    ctx.beginPath();
    ctx.moveTo(cx - Math.cos(needle) * 16, cy - Math.sin(needle) * 16);
    ctx.lineTo(nx, ny);
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    ctx.shadowColor = 'rgba(34,197,94,0.8)';
    ctx.shadowBlur = 8;
    ctx.stroke();
    ctx.shadowBlur = 0;

    ctx.beginPath();
    ctx.arc(cx, cy, 8, 0, Math.PI * 2);
    ctx.fillStyle = '#22c55e';
    ctx.fill();

    ctx.fillStyle = '#e2e8f0';
    ctx.font = 'bold 22px "Segoe UI", Tahoma, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(a.toFixed(1) + '\u00B0', cx, cy + r * 0.72);
  }

  const chartDistCanvas = document.getElementById('chartDist');
  const chartDistCtx = chartDistCanvas.getContext('2d');
  const chartErrorCanvas = document.getElementById('chartError');
  const chartErrorCtx = chartErrorCanvas.getContext('2d');

  function resizeCanvases() {
    [chartDistCanvas, chartErrorCanvas].forEach((c) => {
      c.width = c.clientWidth;
      c.height = c.clientHeight;
    });
    drawCharts();
    drawGauge(gaugeAngle);
  }
  window.addEventListener('resize', resizeCanvases);
  resizeCanvases();

  function niceStep(range, targetTicks) {
    if (range <= 0) return 1;
    const rawStep = range / targetTicks;
    const mag = Math.pow(10, Math.floor(Math.log10(rawStep)));
    const norm = rawStep / mag;
    let step;
    if (norm < 1.5) step = 1;
    else if (norm < 3) step = 2;
    else if (norm < 7) step = 5;
    else step = 10;
    return step * mag;
  }

  function computeYRange(arrays, padFrac, minRange) {
    let min = Infinity, max = -Infinity;
    arrays.forEach((arr) => arr.forEach((v) => {
      if (v < min) min = v;
      if (v > max) max = v;
    }));
    if (!isFinite(min) || !isFinite(max)) { min = 0; max = 1; }
    if (max - min < minRange) {
      const mid = (max + min) / 2;
      min = mid - minRange / 2;
      max = mid + minRange / 2;
    }
    const pad = (max - min) * padFrac;
    return { yMin: min - pad, yMax: max + pad };
  }

  function drawGrid(ctx, w, plotH, padLeft, yMin, yMax, n) {
    ctx.font = '11px "Segoe UI", Tahoma, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';

    const range = yMax - yMin;
    const step = niceStep(range, 5);
    const firstLine = Math.ceil(yMin / step) * step;
    const decimals = step < 1 ? 1 : 0;

    for (let v = firstLine; v <= yMax + step * 0.001; v += step) {
      const y = plotH - ((v - yMin) / range) * plotH;
      ctx.strokeStyle = (Math.abs(v) < step * 0.001) ? '#475569' : '#1e293b';
      ctx.lineWidth = (Math.abs(v) < step * 0.001) ? 1.5 : 1;
      ctx.beginPath();
      ctx.moveTo(padLeft, y);
      ctx.lineTo(w, y);
      ctx.stroke();
      ctx.fillStyle = '#94a3b8';
      ctx.fillText(v.toFixed(decimals), 2, y);
    }

    const vLines = 6;
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let i = 0; i <= vLines; i++) {
      const x = padLeft + (i / vLines) * (w - padLeft);
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, plotH);
      ctx.stroke();
    }

    if (n >= 2) {
      ctx.fillStyle = '#64748b';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'alphabetic';
      for (let i = 0; i <= vLines; i++) {
        const frac = i / vLines;
        const x = padLeft + frac * (w - padLeft);
        const idx = frac * (n - 1);
        const secondsAgo = (n - 1 - idx) * SAMPLE_PERIOD_S;
        ctx.fillText('-' + secondsAgo.toFixed(0) + 's', x, plotH + 12);
      }
      ctx.textAlign = 'left';
    }
  }

  function drawSeries(ctx, arr, color, toX, toY) {
    if (arr.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < arr.length; i++) {
      const x = toX(i, arr.length);
      const y = toY(arr[i]);
      if (i === 0) { ctx.moveTo(x, y); } else { ctx.lineTo(x, y); }
    }
    ctx.stroke();
  }

  function drawChartPanel(canvas, ctx, series, padFrac, minRange) {
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    if (w === 0 || h === 0) return;

    const padLeft = 34, padBottom = 16;
    const plotH = h - padBottom;
    const n = series.reduce((max, s) => Math.max(max, s.data.length), 0);

    const yr = computeYRange(series.map((s) => s.data), padFrac, minRange);
    drawGrid(ctx, w, plotH, padLeft, yr.yMin, yr.yMax, n);

    const toY = (v) => plotH - ((v - yr.yMin) / (yr.yMax - yr.yMin)) * plotH;
    const toX = (i, len) => (len <= 1) ? padLeft : padLeft + (i / (len - 1)) * (w - padLeft);

    series.forEach((s) => drawSeries(ctx, s.data, s.color, toX, toY));
  }

  function drawCharts() {
    drawChartPanel(chartDistCanvas, chartDistCtx, [
      { data: distArr, color: '#38bdf8' },
      { data: spArr, color: '#22c55e' },
    ], 0.15, 4);

    drawChartPanel(chartErrorCanvas, chartErrorCtx, [
      { data: errArr, color: '#22c55e' },
    ], 0.2, 2);
  }

  highlightPreset(15);
  drawGauge(90);
</script>
</body>
</html>
)rawliteral";
///////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);  // Usamos 115200 para graficas mas rapidas

  // Configuracion del ADC del ESP32: 12 bits (0-4095) y rango completo 0-3.3V
  analogReadResolution(12);
  analogSetPinAttenuation(Analog_in, ADC_11db);

  // Configuracion especial para servos en ESP32
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  // ADVERTENCIA DE HARDWARE (causa de una falla real: servo con el tope
  // interno del potenciometro roto, quedo girando solo hacia un lado):
  // (500, 2400) es un rango de pulso MUY cercano a los extremos absolutos
  // que puede pedir la libreria, y muchos servos analogicos hobby NO
  // soportan ese rango completo sin chocar contra su tope mecanico interno.
  // (900, 2100) es un rango mucho mas conservador: casi todos los servos
  // analogicos comunes lo toleran sin llegar a su tope fisico. Se sacrifica
  // algo de rango de giro util a cambio de no volver a romper un servo.
  // IMPORTANTE al reemplazar el servo: antes de confiar en este rango,
  // probarlo por separado con un sketch simple que mueva el servo LENTO de
  // extremo a extremo (0 a 180) escuchando/observando si fuerza o zumba
  // cerca de los bordes; si lo hace, angostar aun mas este rango (ej. a
  // 1000-2000, el estandar RC "seguro" universal) antes de conectarlo a este
  // proyecto.
  myservo.attach(servoPin, 900, 2100);  // Adjunta el servo en el pin definido

  myservo.write(125);        // Posicion inicial del servo, para que la barra quede al medio
  pinMode(Analog_in, INPUT);
  currentTime = millis();

  // --- Punto de acceso WiFi propio del ESP32 ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("Access Point iniciado. SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP del ESP32 (abrir en el navegador): http://");
  Serial.println(WiFi.softAPIP());

  // --- WebSocket para telemetria y comandos ---
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // --- Ruta HTTP principal: sirve la interfaz embebida ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
  Serial.println("Servidor HTTP y WebSocket iniciados.");
}

////////////////////////Comandos por Serial (USB)///////////////////////////
// Permite recalibrar/probar sin conectar al SoftAP (asi la PC no pierde
// internet). Mismos efectos que el WebSocket, procesados en loop().
// Formato de entrada (linea terminada en \n):
//   CMD,setpoint,<cm>
//   CMD,mode,auto|manual
//   CMD,manual_angle,<deg>
// Telemetria extra (ademas del Serial Plotter):
//   TELEM,sp,dist,err,mode,sv,svw,kick,pi
void processSerialCommands() {
  static char lineBuf[64];
  static size_t lineLen = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      lineLen = 0;
      if (strncmp(lineBuf, "CMD,", 4) != 0) continue;

      char *save = nullptr;
      char *tok = strtok_r(lineBuf, ",", &save);  // CMD
      char *type = strtok_r(nullptr, ",", &save);
      char *value = strtok_r(nullptr, ",", &save);
      if (type == nullptr || value == nullptr) continue;

      bool autotuneBusy = (autotuneState == ATS_RUNNING);
      if (strcmp(type, "setpoint") == 0) {
        if (autotuneBusy) continue;
        distance_setpoint = constrain(atof(value), SETPOINT_MIN_CM, SETPOINT_MAX_CM);
      } else if (strcmp(type, "mode") == 0) {
        if (autotuneBusy) continue;
        if (strcmp(value, "manual") == 0) currentMode = MODE_MANUAL;
        else if (strcmp(value, "auto") == 0) currentMode = MODE_AUTO;
      } else if (strcmp(type, "manual_angle") == 0) {
        if (autotuneBusy) continue;
        manual_angle = atof(value);
      }
    } else if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow: descarta linea corrupta
    }
  }
}

void loop() {
  processSerialCommands();

  // Resta con unsigned long: segura ante el desborde (overflow) de millis() cada ~50 dias
  if (millis() - currentTime >= period) {
    currentTime = millis();
    distance = get_dist(100);
    distance = kalmanFilter(distance);  // Suaviza el ruido residual del sensor antes de todo lo demas

    // --- Consumo de los flags de auto-tune pedidos desde el WebSocket ---
    // Se procesan aca (al principio de loop(), en el contexto del task
    // principal) y nunca directamente en el callback del WebSocket, para que
    // toda la logica del auto-tune solo toque 'distance'/'PID_i'/etc. desde
    // un unico contexto de ejecucion (ver comentario en la seccion de
    // auto-tune, mas arriba).
    if (autotuneStartRequested) {
      autotuneStartRequested = false;
      if (autotuneState != ATS_RUNNING) {
        startAutotune(autotuneRequestedPoint);
      }
    }
    if (autotuneCancelRequested) {
      autotuneCancelRequested = false;
      if (autotuneState == ATS_RUNNING) {
        abortAutotune("Cancelado por el usuario.");
      }
    }
    if (autotuneApplyRequested) {
      autotuneApplyRequested = false;
      applyAutotuneResult();
    }

    // --- SETPOINT (sin rampa, ver comentario junto a SETPOINT_MAX_SLEW_CM_PER_S) ---
    // effective_setpoint sigue existiendo como variable separada de
    // distance_setpoint (en vez de fundirlas en una sola) para no tener que
    // tocar el resto del codigo/telemetria que ya la usa, pero ahora salta
    // directo al valor pedido desde la web.
    effective_setpoint = distance_setpoint;

    // Al cambiar el setpoint, limpia estados integrales/friccion para no arrastrar
    // windup del tramo anterior (medido por serial: integral +25 deg al salir de
    // 23cm provocaba sobrepaso enorme hacia 15cm y fallaba el asentamiento).
    static float prev_effective_setpoint = 15.0;
    if (fabs(effective_setpoint - prev_effective_setpoint) > 0.25) {
      smc_integral = 0.0;
      frictionCompStillSinceMs = 0;
      unstickStillSinceMs = 0;
      unstickKickActive = false;
      prev_effective_setpoint = effective_setpoint;
    }

    // "real_error" es el error verdadero (setpoint efectivo - distancia filtrada),
    // SIN zona muerta. Se usa solo para telemetria/grafica, para que el usuario vea
    // el error real incluso cuando el controlador ya lo considera "cero" internamente.
    real_error = effective_setpoint - distance;
    distance_error = real_error;

    // --- ZONA MUERTA (DEADBAND) ---
    // OJO: "distance_error"/"inDeadband" ya NO los usa el controlador SMC de
    // MODE_AUTO (ver bloque mas abajo, que trabaja directo con "real_error" y
    // la capa limite SMC_PHI en su lugar: la zona muerta "dura" de aca abajo
    // era precisamente una de las causas del error residual oscilante
    // observado en hardware, ver comentario junto a SMC_LAMBDA). Se deja el
    // calculo sin borrar solo como referencia/respaldo del esquema anterior.
    bool inDeadband = (distance_error > -DEADBAND && distance_error < DEADBAND);
    if (inDeadband) {
      distance_error = 0;
    }

    if (currentMode == MODE_AUTO) {
      // --- Estimacion de velocidad de la pelota (cm/s), filtrada (EMA) ---
      float raw_velocity = (distance - distance_previous) / (period / 1000.0);
      float velocity_filter_alpha = interpolate3Point(VELOCITY_FILTER_ALPHA_AT_MIN, VELOCITY_FILTER_ALPHA_AT_MID, VELOCITY_FILTER_ALPHA_AT_MAX, effective_setpoint);
      ball_velocity_filtered = velocity_filter_alpha * raw_velocity + (1 - velocity_filter_alpha) * ball_velocity_filtered;

      float errAbs = fabs(real_error);
      float velAbs = fabs(ball_velocity_filtered);

      // Al cambiar setpoint: salir de REGULATE
      static float prev_sp_reg = 15.0f;
      if (fabs(effective_setpoint - prev_sp_reg) > 0.25f) {
        regulateActive = false;
        regulateCount = 0;
        regulateExitCount = 0;
        regulateErrFilt = 0.0f;
        smc_integral = 0.0f;
        prev_sp_reg = effective_setpoint;
      }

      // --- REGULATE: freeze + trim filtrado (hold largo, residual bajo) ---
      bool didRegulate = false;
      if (regulateActive) {
        if (errAbs > HYB_REG_EXIT_CM) {
          if (regulateExitCount < 255) regulateExitCount++;
          if (regulateExitCount >= HYB_REG_EXIT_COUNT) {
            regulateActive = false;
            regulateCount = 0;
            regulateExitCount = 0;
          }
        } else {
          regulateExitCount = 0;
        }
      }
      if (regulateActive) {
          smc_integral = 0.0f;
          frictionCompStillSinceMs = 0;
          unstickStillSinceMs = 0;
          unstickKickActive = false;
          regulateErrFilt = 0.94f * regulateErrFilt + 0.06f * real_error;
          float eF = regulateErrFilt;
          float eFabs = fabs(eF);
          if (eFabs > HYB_REG_TRIM_DEAD && velAbs < 0.65f) {
            float gain = HYB_REG_TRIM_GAIN;
            float cap = HYB_REG_TRIM_MAX;
            if (eFabs > 0.28f) { gain *= 1.45f; cap *= 1.25f; }
            float trim = constrain(gain * eF, -cap, cap);
            float candidate = constrain(
              regulateAngle - trim,
              regulateAngleFreeze - HYB_REG_TRIM_SPAN,
              regulateAngleFreeze + HYB_REG_TRIM_SPAN);
            regulateAngle = constrain(
              candidate,
              (float)(SERVO_OUT_MIN + SERVO_OFFSET),
              (float)(SERVO_OUT_MAX + SERVO_OFFSET));
          }
          PID_p = 0.0f;
          PID_i = 0.0f;
          PID_d_filtered = 0.0f;
          PID_total = 0.0f;
          servo_angle_actual = regulateAngle;
          writeServoLimited(regulateAngle);
          didRegulate = true;
      }

      if (!didRegulate) {
      // --- SMC: cerca del SP sin -v (evita invertir en 21cm) ---
      float smc_lambda = interpolate3Point(SMC_LAMBDA_AT_MIN, SMC_LAMBDA_AT_MID, SMC_LAMBDA_AT_MAX, effective_setpoint);
      float s = (errAbs < 1.5f)
                  ? (smc_lambda * real_error)
                  : (smc_lambda * real_error - ball_velocity_filtered);
      float satRatio = constrain(s / SMC_PHI, -1.0, 1.0);
      float smc_max_use = interpolate3Point(32.0f, 40.0f, SMC_MAX_DEG, effective_setpoint);
      float smc_deg = smc_max_use * satRatio;

      float smc_integral_limit = interpolate3Point(
        SMC_INTEGRAL_LIMIT_AT_MIN, SMC_INTEGRAL_LIMIT_AT_MID, SMC_INTEGRAL_LIMIT_AT_MAX,
        effective_setpoint);
      if (errAbs < 0.42f) {
        smc_integral *= 0.90f;
      } else if (fabs(s) < SMC_PHI) {
        smc_integral += SMC_KI * real_error * (period / 1000.0);
        smc_integral = constrain(smc_integral, -smc_integral_limit, smc_integral_limit);
      } else {
        smc_integral *= 0.9;
      }

      float friction_comp = 0.0;
      bool nearlyStopped = (velAbs < FRICTION_COMP_VEL_THRESHOLD);
      if (nearlyStopped) {
        if (frictionCompStillSinceMs == 0) frictionCompStillSinceMs = millis();
      } else {
        frictionCompStillSinceMs = 0;
      }
      bool sustainedStill = (frictionCompStillSinceMs != 0) &&
                             (millis() - frictionCompStillSinceMs >= FRICTION_COMP_MIN_STILL_MS);
      if (sustainedStill && errAbs > FRICTION_COMP_ERROR_THRESHOLD) {
        float friction_comp_deg = interpolate3Point(
          FRICTION_COMP_DEG_AT_MIN, FRICTION_COMP_DEG_AT_MID, FRICTION_COMP_DEG_AT_MAX,
          effective_setpoint);
        float excess = errAbs - FRICTION_COMP_ERROR_THRESHOLD;
        float frictionRamp = constrain(excess / 1.5, 0.0, 1.0);
        friction_comp = (real_error > 0 ? 1.0 : -1.0) * friction_comp_deg * frictionRamp;
      }

      float total_correction_deg = smc_deg + smc_integral + friction_comp;

      PID_p = smc_deg;
      PID_i = smc_integral;
      PID_d_filtered = friction_comp;
      PID_total = total_correction_deg;

      float servo_deviation_deg = constrain(-total_correction_deg, -(SERVO_OUT_MAX - 90), SERVO_OUT_MAX - 90);
      servo_angle_actual = constrain(90 + servo_deviation_deg, SERVO_OUT_MIN, SERVO_OUT_MAX) + SERVO_OFFSET;

      // Entrada a REGULATE: preferir |e| bajo; si no, banda ancha
      bool enterGood = (errAbs <= HYB_REG_ENTER_GOOD_CM && velAbs <= HYB_REG_ENTER_VEL);
      bool enterWide = (errAbs <= HYB_REG_ENTER_CM && velAbs <= HYB_REG_ENTER_VEL);
      uint8_t needCount = enterGood ? HYB_REG_COUNT_GOOD : HYB_REG_COUNT;
      if (enterGood || enterWide) {
        if (regulateCount < 255) regulateCount++;
        if (regulateCount >= needCount) {
          regulateActive = true;
          float tip = 0.0f;
          if (effective_setpoint >= 18.0f) {
            if (real_error > 0.22f) tip = -1.2f;
            else if (real_error < -0.22f) tip = 1.2f;
          }
          regulateAngle = constrain(
            servoWriteAngleLast + tip,
            (float)(SERVO_OUT_MIN + SERVO_OFFSET),
            (float)(SERVO_OUT_MAX + SERVO_OFFSET));
          regulateAngleFreeze = regulateAngle;
          regulateErrFilt = real_error;
          regulateExitCount = 0;
          smc_integral = 0.0f;
          writeServoLimited(regulateAngle);
        } else if (errAbs < 0.65f) {
          writeServoLimited(servo_angle_actual);
        } else {
          writeServoLimited(applyDither(applyUnstickKick(servo_angle_actual)));
        }
      } else {
        if (errAbs > HYB_REG_ENTER_CM + 0.18f) regulateCount = 0;
        if (errAbs < 0.65f) {
          writeServoLimited(servo_angle_actual);
        } else {
          writeServoLimited(applyDither(applyUnstickKick(servo_angle_actual)));
        }
      }
      }  // fin if (!didRegulate)
    } else if (currentMode == MODE_MANUAL) {
      PID_i = 0;
      smc_integral = 0;
      ball_velocity_filtered = 0;
      frictionCompStillSinceMs = 0;
      regulateActive = false;
      regulateCount = 0;
      regulateExitCount = 0;
      regulateErrFilt = 0.0f;
      servo_angle_actual = constrain(manual_angle, SERVO_OUT_MIN, SERVO_OUT_MAX) + SERVO_OFFSET;
      writeServoLimited(applyDither(servo_angle_actual));
    } else {
      // MODE_AUTOTUNE: control por rele (ver runAutotuneStep() y la seccion de
      // auto-tune mas arriba). No usa kp/ki/kd ni dither: el objetivo es medir
      // la oscilacion natural del sistema, no controlarlo con suavidad.
      runAutotuneStep();
    }

    real_error_previous = real_error;
    distance_previous = distance;

    // Imprimimos los datos para poder verlos en el "Serial Plotter"
    // TRUCO GRAFICO: Imprimimos un 0 y un 40 fijos para bloquear el auto-zoom de Arduino
    Serial.print("Min:0,");
    Serial.print("Max:40,");
    Serial.print("Setpoint:"); Serial.print(effective_setpoint); Serial.print(",");
    Serial.print("Distancia:"); Serial.println(distance);

    // Linea parseable por scripts de calibracion por USB (ver serial_settle_test.py)
    const char *modeStr = (currentMode == MODE_AUTO) ? "auto" : (currentMode == MODE_MANUAL ? "manual" : "autotune");
    Serial.print("TELEM,");
    Serial.print(effective_setpoint, 2); Serial.print(',');
    Serial.print(distance, 2); Serial.print(',');
    Serial.print(real_error, 2); Serial.print(',');
    Serial.print(modeStr); Serial.print(',');
    Serial.print(servo_angle_actual, 1); Serial.print(',');
    Serial.print(servoWriteAngleLast, 1); Serial.print(',');
    Serial.print(unstickKickActive ? 1 : 0); Serial.print(',');
    Serial.println(PID_i, 1);

    // Telemetria via WebSocket limitada a ~10Hz (cada 2 iteraciones del lazo de 50ms)
    broadcastCounter++;
    if (broadcastCounter >= 2) {
      broadcastCounter = 0;
      broadcastTelemetry();
    }
  }
}

// Lee el sensor Sharp GP2Y0A21 y devuelve la distancia estimada en cm
float get_dist(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(Analog_in);
  }
  // Promedio de la lectura cruda del ESP32 (Rango 0 - 4095)
  float adc_esp32 = (float)sum / n;

  // --- ADAPTACION MATEMATICA ESP32 A ARDUINO ---
  // Calculamos que voltaje real leyo el ESP32 (Referencia 3.3V)
  float voltaje_real = (adc_esp32 / 4095.0) * 3.3;

  // Simulamos que lectura de ADC habria dado el Arduino Uno (Referencia 5V) con ese voltaje.
  // Esta es la clave para que la formula original de Electronoobs siga funcionando.
  float adc_simulado_arduino = (voltaje_real / 5.0) * 1023.0;

  // Proteccion contra division por cero / potencia invalida en la formula de linealizacion
  if (adc_simulado_arduino < 1) { adc_simulado_arduino = 1; }

  // Formula original de linealizacion del autor
  float distance_cm = 17569.7 * pow(adc_simulado_arduino, -1.2062);

  return distance_cm;
}

// Envia por WebSocket a todos los clientes conectados un JSON con la telemetria actual
void broadcastTelemetry() {
  if (ws.count() == 0) return;  // Evita construir el JSON si no hay nadie conectado

  StaticJsonDocument<512> doc;
  doc["distance"] = distance;
  doc["setpoint"] = effective_setpoint;  // Setpoint YA suavizado por la rampa: asi el usuario ve la rampa graficada en tiempo real
  doc["error"] = real_error;  // Error real (sin zona muerta), no el interno del PID
  doc["mode"] = (currentMode == MODE_AUTO) ? "auto" : (currentMode == MODE_MANUAL ? "manual" : "autotune");
  doc["servo_angle"] = servo_angle_actual;  // Angulo "limpio" calculado por el PID (SIN dither/golpe anti-atasco, ver writeServoLimited())

  // --- Campos de depuracion (diagnostico de por que el auto-tune/PID no
  // logran mover la pelota): el resto de la telemetria muestra el angulo
  // "limpio" del PID, pero NO el que realmente se escribe al servo (que
  // incluye el limitador de velocidad, el dither y el golpe anti-atasco), ni
  // si el golpe esta activo en este momento, ni los valores de ganancia
  // realmente usados por el gain scheduling. Sin esto no se puede confirmar
  // desde afuera si el golpe se esta disparando o no.
  doc["servo_angle_written"] = servoWriteAngleLast;  // Angulo REAL escrito al servo (post limitador/dither/golpe)
  doc["unstick_active"] = unstickKickActive;
  doc["pid_p"] = PID_p;
  doc["pid_i"] = PID_i;
  doc["pid_d_filtered"] = PID_d_filtered;
  // kp_used/kd_used quedan por compatibilidad con herramientas de debug
  // previas, pero ya NO reflejan el controlador SMC actual (que no usa
  // kp/kd): se reemplaza su contenido por las variables relevantes del nuevo
  // controlador (ver bloque MODE_AUTO en loop() y SMC_* mas arriba).
  doc["kp_used"] = ball_velocity_filtered;  // "kp_used" reutilizado: velocidad estimada de la pelota (cm/s)
  doc["kd_used"] = smc_integral;            // "kd_used" reutilizado: aporte integral del SMC (grados)

  // --- Estado del auto-tune (siempre presente, para que la interfaz pueda
  // mostrar/ocultar el panel correspondiente sin depender de mensajes previos) ---
  doc["autotune_state"] = autotuneStateToStr(autotuneState);
  doc["autotune_point"] = autotunePointToStr(autotunePoint);
  doc["autotune_message"] = autotuneMessage;
  doc["autotune_elapsed_s"] = (autotuneState == ATS_RUNNING) ? (millis() - autotuneStartMs) / 1000.0 : 0;
  if (autotuneState == ATS_DONE) {
    doc["autotune_kp"] = autotuneResultKp;
    doc["autotune_ki"] = autotuneResultKi;
    doc["autotune_kd"] = autotuneResultKd;
  }

  char buffer[512];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  ws.textAll(buffer, len);
}

// Procesa un mensaje JSON entrante desde el navegador (comandos de setpoint/modo/angulo manual)
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) {
    return;  // Ignora mensajes fragmentados o binarios, no se usan en este proyecto
  }

  data[len] = 0;  // Null-terminate para poder tratarlo como cadena C
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, (char *)data);
  if (err) {
    Serial.println("Error al parsear JSON del WebSocket");
    return;
  }

  const char *type = doc["type"];
  if (type == nullptr) return;

  // Mientras el auto-tune esta en curso (ATS_RUNNING), se ignoran los comandos
  // normales de setpoint/modo/angulo manual: la unica forma de interrumpirlo
  // es "autotune_cancel". Esto evita que el sistema quede en un estado raro
  // si, por ejemplo, otro cliente conectado cambia de modo a mitad de la
  // prueba (ver comentario de 'autotuneState volatile' en su declaracion).
  bool autotuneBusy = (autotuneState == ATS_RUNNING);

  if (strcmp(type, "setpoint") == 0) {
    if (autotuneBusy) return;
    float value = doc["value"];
    distance_setpoint = constrain(value, SETPOINT_MIN_CM, SETPOINT_MAX_CM);
  } else if (strcmp(type, "mode") == 0) {
    if (autotuneBusy) return;
    const char *value = doc["value"];
    if (value == nullptr) return;
    if (strcmp(value, "manual") == 0) {
      currentMode = MODE_MANUAL;
    } else if (strcmp(value, "auto") == 0) {
      currentMode = MODE_AUTO;
    }
  } else if (strcmp(type, "manual_angle") == 0) {
    if (autotuneBusy) return;
    float value = doc["value"];
    manual_angle = value;
  } else if (strcmp(type, "autotune_start") == 0) {
    if (autotuneBusy) return;
    const char *pointStr = doc["point"];
    if (pointStr == nullptr) return;
    AutotunePoint p = ATP_MID;
    if (strcmp(pointStr, "min") == 0) p = ATP_MIN;
    else if (strcmp(pointStr, "max") == 0) p = ATP_MAX;
    else if (strcmp(pointStr, "mid") == 0) p = ATP_MID;
    else return;
    autotuneRequestedPoint = p;
    autotuneStartRequested = true;
  } else if (strcmp(type, "autotune_cancel") == 0) {
    autotuneCancelRequested = true;
  } else if (strcmp(type, "autotune_apply") == 0) {
    autotuneApplyRequested = true;
  }
}

// Callback de eventos del WebSocket (conexion, desconexion, datos entrantes)
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("Cliente WebSocket #%u conectado desde %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("Cliente WebSocket #%u desconectado\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}
