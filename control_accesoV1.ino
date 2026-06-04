
/*

  Sistema de Control de Acceso con Cerradura Inteligente

  Universidad del Cauca - Arquitectura Computacional

  Hardware: Arduino Mega, Teclado 4x4, LCD 16x2, LED RGB,

            Termistor, LDR, Sensor Hall, Microfono, RFID RC522, Buzzer, Servo

*/


#include <Keypad.h>

#include <LiquidCrystal.h>

#include <EEPROM.h>

#include <SPI.h>

#include <MFRC522.h>

#include <Servo.h>

#include "AsyncTaskLib.h"

#include "StateMachineLib.h"


#define DEBUG(a) Serial.print(millis()); Serial.print(": "); Serial.println(a)


// ── PINES ──────────────────────────────────────────────────────────────────

#define PIN_RED     22

#define PIN_GREEN   26

#define PIN_BLUE    24

#define PIN_BUZZER   6

#define PIN_SERVO    8

#define PIN_BOTON   10


#define PIN_TEMP    A8

#define PIN_LUZ     A7

#define PIN_HALL    A6

#define PIN_MIC_AN  A5

#define PIN_MIC_DIG  7


#define PIN_RFID_SS 53

#define PIN_RFID_RST 9


// ── TECLADO 4x4 ────────────────────────────────────────────────────────────

const byte ROWS = 4;

const byte COLS = 4;

char keys[ROWS][COLS] = {

  {'1','2','3','A'},

  {'4','5','6','B'},

  {'7','8','9','C'},

  {'*','0','#','D'}

};

byte rowPins[ROWS] = {25, 27, 29, 31};

byte colPins[COLS]  = {33, 35, 37, 39};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


// ── LCD ────────────────────────────────────────────────────────────────────

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// ── RFID ───────────────────────────────────────────────────────────────────

MFRC522 mfrc522(PIN_RFID_SS, PIN_RFID_RST);


// ── SERVO ──────────────────────────────────────────────────────────────────

Servo myservo;

#define SERVO_ABIERTO 90

#define SERVO_CERRADO  0


// ── CONSTANTES ─────────────────────────────────────────────────────────────

#define NUM_INTENTOS  3

#define NUM_DIGIT     4

#define MAX_USUARIOS 10

#define MAX_USOS      4


#define UMBRAL_TEMP_ALTO  30

#define UMBRAL_TEMP_BAJO  20

#define UMBRAL_LUZ_BAJO  100

#define UMBRAL_HALL      512

#define UMBRAL_MIC       600

#define UMBRAL_MIC_VECES   3


#define T_PUERTA_ABIERTA 3000

#define T_ALARMA_MAX    12000

#define T_ALARMA_AMB     4000

#define T_ALARMA_INT     2000


// ── EEPROM: ESTRUCTURA DE USUARIO ─────────────────────────────────────────

enum Rol { Seguridad, Operario, Coordinador, Gerente };


struct Usuario {

  char nombre[20];

  int  permiso[2];

  char clave[5];

  char historial[4][5];

  byte usos;

  byte uid[4];

  Rol  perfil;

};


// ── ESTADOS FSM ────────────────────────────────────────────────────────────

enum State {

  INICIO            = 0,

  CONFIG            = 1,

  MONITOR_AMBIENTAL = 2,

  MONITOR_INTRUSOS  = 3,

  ALARMA            = 4,

  BLOQUEO           = 5

};


enum Input {

  INPUT_CLAVE_OK   = 0,

  INPUT_CLAVE_MAL  = 1,

  INPUT_BLOQUEADO  = 2,

  INPUT_SENSOR_MAL = 3,

  INPUT_ALARMA_AMB = 4,

  INPUT_ALARMA_INT = 5,

  INPUT_RESET      = 6,

  INPUT_TIMEOUT    = 7,

  INPUT_UNKNOWN    = 8

};


StateMachine stateMachine(6, 15);

Input inputFSM = INPUT_UNKNOWN;


// ── VARIABLES GLOBALES ─────────────────────────────────────────────────────

unsigned char idx = 0;

unsigned char cont_intentos = 0;

unsigned char cont_alarmas  = 0;

unsigned char cont_mic      = 0;

int  usuario_actual = -1;


char clave_user[5];


// hora simulada

byte horaActual      = 0;

byte minutoActual    = 0;

bool horaConfigurada = false;

byte horaBase        = 0;

byte minBase         = 0;

unsigned long tInicioSistema = 0;


// timers manuales para estados

unsigned long tEntradaEstado  = 0;

unsigned long tPrimeraAlarma  = 0;

unsigned long tPuertaAbierta  = 0;

unsigned long tUltimoBoton    = 0;

bool puertaYaCerrada = false;


// ingreso hora

char bufHora[5];

byte idxHora = 0;

bool lcdHoraMostrado = false;


// cambio de clave

bool cambioClaveActivo = false;

char claveNueva1[5];

char claveNueva2[5];

byte idxNueva = 0;

byte faseClave = 0;


// sensores leidos por tasks

float tempLeida = 0;

int   luzLeida  = 0;

int   hallLeido = 0;

int   micLeido  = 0;

int   micDig    = 0;


// parpadeo led rojo (no bloqueante)

bool  ledRojoState  = false;

unsigned long tLedRojo = 0;


// estado antes de alarma (capturado en SetOnLeaving)

State estadoAntesDeAlarma = MONITOR_AMBIENTAL;


// timers no bloqueantes (reemplazo de delay)

unsigned long tMsgInicio     = 0;

bool          msgInicioActivo = false;

byte          msgInicioTipo   = 0;
// 0=fuera franja, 1=clave incorrecta, 2=hora ok, 3=hora invalida, 4=tarjeta invalida

unsigned long tMsgConfig     = 0;

bool          msgConfigActivo = false;

byte          msgConfigTipo   = 0;
// 0=no coinciden, 1=clave ya usada, 2=clave cambiada

unsigned long tMsgIntrusos   = 0;

bool          msgIntrusosActivo = false;


// ── RFID ───────────────────────────────────────────────────────────────────

byte uidLeido[4];


bool isEqualArray(byte* a, byte* b, int len) {

  for (int i = 0; i < len; i++) {

    if (a[i] != b[i]) return false;

  }

  return true;

}


// ── EEPROM: GUARDAR Y LEER USUARIO ────────────────────────────────────────

void guardarUsuario(int idx, Usuario u) {

  EEPROM.put(idx * sizeof(Usuario), u);

}


void leerUsuario(int idx, Usuario &u) {

  EEPROM.get(idx * sizeof(Usuario), u);

}


bool claveRepetida(Usuario u, char* clave) {

  for (int i = 0; i < 4; i++) {

    if (strcmp(u.historial[i], clave) == 0) return true;

  }

  return false;

}


void cambiarClave(int idx, char* nueva) {

  Usuario u;

  leerUsuario(idx, u);

  for (int i = 3; i > 0; i--) {

    strcpy(u.historial[i], u.historial[i-1]);

  }

  strcpy(u.historial[0], u.clave);

  strcpy(u.clave, nueva);

  u.usos = 0;

  guardarUsuario(idx, u);

}


int buscarPorClave(char* clave) {

  for (int i = 0; i < MAX_USUARIOS; i++) {

    Usuario u;

    leerUsuario(i, u);

    if (strcmp(u.clave, clave) == 0) return i;

  }

  return -1;

}


int buscarPorUID(byte* uid) {

  for (int i = 0; i < MAX_USUARIOS; i++) {

    Usuario u;

    leerUsuario(i, u);

    if (isEqualArray(u.uid, uid, 4) &&

        !(u.uid[0]==0 && u.uid[1]==0 && u.uid[2]==0 && u.uid[3]==0)) {

      return i;

    }

  }

  return -1;

}


void cargarUsuariosIniciales() {

  Usuario u;


  strcpy(u.nombre, "Gerente1");    strcpy(u.clave, "1111");

  u.permiso[0]=8; u.permiso[1]=23; u.perfil=Gerente;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(0, u);


  strcpy(u.nombre, "Coord1");      strcpy(u.clave, "2222");

  u.permiso[0]=7; u.permiso[1]=18; u.perfil=Coordinador;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(1, u);


  strcpy(u.nombre, "Coord2");      strcpy(u.clave, "2233");

  u.permiso[0]=7; u.permiso[1]=18; u.perfil=Coordinador;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(2, u);


  strcpy(u.nombre, "Operario1");   strcpy(u.clave, "3333");

  u.permiso[0]=6; u.permiso[1]=16; u.perfil=Operario;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(3, u);


  strcpy(u.nombre, "Operario2");   strcpy(u.clave, "3344");

  u.permiso[0]=6; u.permiso[1]=16; u.perfil=Operario;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(4, u);


  strcpy(u.nombre, "Operario3");   strcpy(u.clave, "3355");

  u.permiso[0]=6; u.permiso[1]=16; u.perfil=Operario;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(5, u);


  strcpy(u.nombre, "Operario4");   strcpy(u.clave, "3366");

  u.permiso[0]=6; u.permiso[1]=16; u.perfil=Operario;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(6, u);


  strcpy(u.nombre, "Operario5");   strcpy(u.clave, "3377");

  u.permiso[0]=6; u.permiso[1]=16; u.perfil=Operario;

  u.usos=0; u.uid[0]=0; u.uid[1]=0; u.uid[2]=0; u.uid[3]=0;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(7, u);


  strcpy(u.nombre, "Seguridad1");  strcpy(u.clave, "4444");

  u.permiso[0]=0; u.permiso[1]=23; u.perfil=Seguridad;

  u.usos=0; u.uid[0]=0x56; u.uid[1]=0x34; u.uid[2]=0xDA; u.uid[3]=0x73;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(8, u);


  strcpy(u.nombre, "Seguridad2");  strcpy(u.clave, "4455");

  u.permiso[0]=0; u.permiso[1]=23; u.perfil=Seguridad;

  u.usos=0; u.uid[0]=0x43; u.uid[1]=0x89; u.uid[2]=0x4F; u.uid[3]=0x2E;

  for(int j=0;j<4;j++) strcpy(u.historial[j],"0000");

  guardarUsuario(9, u);


  DEBUG("Usuarios inicializados en EEPROM");

}


// ── SENSORES: FUNCIONES PARA TASKS ────────────────────────────────────────

void leer_temperatura() {

  float Vo = analogRead(PIN_TEMP);

  float R2  = 10000.0 * (1023.0 / (float)Vo - 1.0);

  float logR2 = log(R2);

  tempLeida = (1.0 / (0.001129148 + 0.000234125*logR2 + 0.0000000876741*logR2*logR2*logR2)) - 273.15;

  Serial.print("Temp: "); Serial.println(tempLeida);

}


void leer_luz() {

  luzLeida = analogRead(PIN_LUZ);

  Serial.print("Luz: "); Serial.println(luzLeida);

}


void leer_hall() {

  hallLeido = analogRead(PIN_HALL);

  Serial.print("Hall: "); Serial.println(hallLeido);

}


void leer_microfono() {

  micLeido = analogRead(PIN_MIC_AN);

  micDig   = digitalRead(PIN_MIC_DIG);

  Serial.print("Mic: "); Serial.println(micLeido);

}


// ── TASKS SENSORES ────────────────────────────────────────────────────────

AsyncTask Task1(1800, true, leer_temperatura);

AsyncTask Task2(1700, true, leer_luz);

AsyncTask Task3(2400, true, leer_hall);

AsyncTask Task4(300,  true, leer_microfono);


// ── TASKS MENSAJES (reemplazo de delay) ───────────────────────────────────

// Cada uno dispara una funcion al vencer el tiempo

// Se usan como one-shot (false = no repetir)


void onFueraFranja() {

  inputFSM = INPUT_UNKNOWN;

  lcd.clear();

  lcd.print("Ingrese clave:");

}


void onClaveIncorrecta() {

  if (cont_intentos >= NUM_INTENTOS) {

    setLED(true, false, false);

    inputFSM = INPUT_BLOQUEADO;

  } else {

    setLED(false, false, true);

    lcd.clear();

    lcd.print("Intentos:");

    lcd.setCursor(9, 0);

    lcd.print(cont_intentos);

    lcd.print("/");

    lcd.print(NUM_INTENTOS);

    AsyncTask* t = nullptr; // se relanza TaskMsgB para mostrar intentos 1 seg mas

    // simplificado: volvemos directo a pedir clave

    setLED(false, false, false);

    lcd.clear();

    lcd.print("Ingrese clave:");

  }

}


void onHoraOk() {

  lcd.clear();

  lcd.print("Ingrese clave:");

}


void onHoraInvalida() {

  idxHora = 0;

  lcd.clear();

  lcd.print("Ingrese hora:");

  lcd.setCursor(0, 1);

  lcd.print("HHMM: ");

}


void onTarjetaInvalida() {

  lcd.clear();

  lcd.print("Ingrese clave:");

}


void onNoCoinciden() {

  faseClave = 1;

  idxNueva  = 0;

  lcd.clear();

  lcd.print("Nueva clave:");

}


void onClaveYaUsada() {

  faseClave = 1;

  idxNueva  = 0;

  lcd.clear();

  lcd.print("Nueva clave:");

}


void onClaveCambiada() {

  cambioClaveActivo = false;

  faseClave = 0;

  idxNueva  = 0;

  lcd.clear();

  lcd.print("Puerta cerrada");

  lcd.setCursor(0, 1);

  lcd.print("A:clave  *:salir");

}


void onIntrusoDetectado() {

  inputFSM = INPUT_SENSOR_MAL;

}


void on3Alarmas() {

  cont_alarmas = 0;

  inputFSM = INPUT_RESET;

}


AsyncTask TaskFueraFranja  (1500, false, onFueraFranja);

AsyncTask TaskClaveInc     (1000, false, onClaveIncorrecta);

AsyncTask TaskHoraOk       (800,  false, onHoraOk);

AsyncTask TaskHoraInv      (800,  false, onHoraInvalida);

AsyncTask TaskTarjetaInv   (1000, false, onTarjetaInvalida);

AsyncTask TaskNoCoinciden  (1000, false, onNoCoinciden);

AsyncTask TaskClaveYaUsada (1000, false, onClaveYaUsada);

AsyncTask TaskClaveCambiada(1200, false, onClaveCambiada);

AsyncTask TaskIntrusoDet   (500,  false, onIntrusoDetectado);

AsyncTask Task3Alarmas     (800,  false, on3Alarmas);


// ── HORA SIMULADA ──────────────────────────────────────────────────────────

void actualizarHora() {

  if (!horaConfigurada) return;

  unsigned long elapsed = (millis() - tInicioSistema) / 1000UL;

  unsigned long totalMin = (unsigned long)horaBase * 60 + minBase + elapsed / 60;

  horaActual   = (byte)((totalMin / 60) % 24);

  minutoActual = (byte)(totalMin % 60);

}


bool dentroFranja(int horaI, int horaF) {

  return (horaActual >= horaI && horaActual < horaF);

}


// ── LED RGB ────────────────────────────────────────────────────────────────

void setLED(bool r, bool g, bool b) {

  digitalWrite(PIN_RED,   r ? HIGH : LOW);

  digitalWrite(PIN_GREEN, g ? HIGH : LOW);

  digitalWrite(PIN_BLUE,  b ? HIGH : LOW);

}


void apagarTodo() {

  setLED(false, false, false);

  digitalWrite(PIN_BUZZER, HIGH);

}


void parpadeoRojo(unsigned long onTime, unsigned long offTime) {

  unsigned long ahora = millis();

  unsigned long intervalo = ledRojoState ? onTime : offTime;

  if (ahora - tLedRojo >= intervalo) {

    ledRojoState = !ledRojoState;

    digitalWrite(PIN_RED, ledRojoState ? HIGH : LOW);

    tLedRojo = ahora;

  }

}


// ── AUTENTICACION ──────────────────────────────────────────────────────────

void autenticarUsuario(int idx_u) {

  Usuario u;

  leerUsuario(idx_u, u);


  if (horaConfigurada && !dentroFranja(u.permiso[0], u.permiso[1])) {

    lcd.clear();

    lcd.print("Acceso denegado");

    lcd.setCursor(0, 1);

    lcd.print("Fuera de franja");

    TaskFueraFranja.Start();

    return;

  }


  usuario_actual = idx_u;

  cont_intentos  = 0;

  idx            = 0;


  u.usos++;

  guardarUsuario(idx_u, u);


  if (u.usos >= MAX_USOS) {

    cambioClaveActivo = true;

  } else {

    cambioClaveActivo = false;

  }


  inputFSM = INPUT_CLAVE_OK;

}


// ── ACCIONES DE ESTADOS ────────────────────────────────────────────────────

void entrarInicio() {

  DEBUG("-> INICIO");

  apagarTodo();

  myservo.write(SERVO_CERRADO);

  usuario_actual = -1;

  idx            = 0;

  idxHora        = 0;

  lcdHoraMostrado = false;

  memset(clave_user, 0, sizeof(clave_user));

  tEntradaEstado = millis();


  if (!horaConfigurada) {

    lcd.clear();

    lcd.print("Ingrese hora:");

    lcd.setCursor(0, 1);

    lcd.print("HHMM: ");

  } else {

    lcd.clear();

    lcd.print("Ingrese clave:");

  }

}


void salirInicio() {

  DEBUG("Saliendo INICIO");

}


void entrarConfig() {

  DEBUG("-> CONFIG");

  tEntradaEstado   = millis();

  tPuertaAbierta   = millis();

  puertaYaCerrada  = false;

  idxNueva         = 0;

  faseClave        = 0;


  setLED(false, true, false);

  myservo.write(SERVO_ABIERTO);


  Usuario u;

  leerUsuario(usuario_actual, u);


  lcd.clear();

  lcd.print("Bienvenido:");

  lcd.setCursor(0, 1);

  lcd.print(u.nombre);

  DEBUG(u.nombre);

}


void salirConfig() {

  DEBUG("Saliendo CONFIG");

  myservo.write(SERVO_CERRADO);

  apagarTodo();

  cambioClaveActivo = false;

  idxNueva = 0;

  faseClave = 0;

}


void entrarMonitorAmbiental() {

  DEBUG("-> MONITOR AMBIENTAL");

  apagarTodo();

  tEntradaEstado = millis();

  lcd.clear();

  lcd.print("AMB ");

  Task1.Start();

  Task2.Start();

}


void salirMonitorAmbiental() {

  DEBUG("Saliendo MONITOR AMBIENTAL");

  estadoAntesDeAlarma = MONITOR_AMBIENTAL;

  Task1.Stop();

  Task2.Stop();

}


void entrarMonitorIntrusos() {

  DEBUG("-> MONITOR INTRUSOS");

  apagarTodo();

  cont_mic       = 0;

  tEntradaEstado = millis();

  lcd.clear();

  lcd.print("INTRUSOS");

  Task3.Start();

  Task4.Start();

}


void salirMonitorIntrusos() {

  DEBUG("Saliendo MONITOR INTRUSOS");

  estadoAntesDeAlarma = MONITOR_INTRUSOS;

  Task3.Stop();

  Task4.Stop();

}


void entrarAlarma() {

  DEBUG("-> ALARMA");

  if (cont_alarmas == 0) tPrimeraAlarma = millis();

  cont_alarmas++;

  // estadoAntesDeAlarma ya fue capturado en salirMonitorAmbiental/salirMonitorIntrusos

  tEntradaEstado = millis();

  ledRojoState   = false;

  tLedRojo       = millis();

  lcd.clear();

  lcd.print("!!! ALARMA !!!");

}


void salirAlarma() {

  DEBUG("Saliendo ALARMA");

  apagarTodo();

}


void entrarBloqueo() {

  DEBUG("-> BLOQUEO");

  tEntradaEstado = millis();

  ledRojoState   = false;

  tLedRojo       = millis();

  lcd.clear();

  lcd.print("BLOQUEADO");

  lcd.setCursor(0, 1);

  lcd.print("Pres. boton");

}


void salirBloqueo() {

  DEBUG("Saliendo BLOQUEO");

  apagarTodo();

  cont_intentos = 0;

}


// ── CONFIGURAR MAQUINA DE ESTADOS ─────────────────────────────────────────

void setupStateMachine() {

  stateMachine.AddTransition(INICIO,            CONFIG,            []() { return inputFSM == INPUT_CLAVE_OK;   });

  stateMachine.AddTransition(INICIO,            BLOQUEO,           []() { return inputFSM == INPUT_BLOQUEADO;  });

  stateMachine.AddTransition(CONFIG,            INICIO,            []() { return inputFSM == INPUT_RESET;      });

  stateMachine.AddTransition(CONFIG,            MONITOR_AMBIENTAL, []() { return inputFSM == INPUT_TIMEOUT;    });

  stateMachine.AddTransition(MONITOR_AMBIENTAL, CONFIG,            []() { return inputFSM == INPUT_CLAVE_OK;   });

  stateMachine.AddTransition(MONITOR_AMBIENTAL, MONITOR_INTRUSOS,  []() { return inputFSM == INPUT_TIMEOUT;    });

  stateMachine.AddTransition(MONITOR_AMBIENTAL, ALARMA,            []() { return inputFSM == INPUT_SENSOR_MAL; });

  stateMachine.AddTransition(MONITOR_INTRUSOS,  CONFIG,            []() { return inputFSM == INPUT_CLAVE_OK;   });

  stateMachine.AddTransition(MONITOR_INTRUSOS,  MONITOR_AMBIENTAL, []() { return inputFSM == INPUT_TIMEOUT;    });

  stateMachine.AddTransition(MONITOR_INTRUSOS,  ALARMA,            []() { return inputFSM == INPUT_SENSOR_MAL; });

  stateMachine.AddTransition(ALARMA,            INICIO,            []() { return inputFSM == INPUT_RESET;      });

  stateMachine.AddTransition(ALARMA,            MONITOR_AMBIENTAL, []() { return inputFSM == INPUT_ALARMA_AMB; });

  stateMachine.AddTransition(ALARMA,            MONITOR_INTRUSOS,  []() { return inputFSM == INPUT_ALARMA_INT; });

  stateMachine.AddTransition(BLOQUEO,           INICIO,            []() { return inputFSM == INPUT_RESET;      });

  // CONFIG -> MONITOR_AMBIENTAL ya existe con INPUT_TIMEOUT (tecla *)

  // se agrega MONITOR_INTRUSOS -> CONFIG con INPUT_CLAVE_OK (tecla #) ya esta arriba


  stateMachine.SetOnEntering(INICIO,            entrarInicio);

  stateMachine.SetOnEntering(CONFIG,            entrarConfig);

  stateMachine.SetOnEntering(MONITOR_AMBIENTAL, entrarMonitorAmbiental);

  stateMachine.SetOnEntering(MONITOR_INTRUSOS,  entrarMonitorIntrusos);

  stateMachine.SetOnEntering(ALARMA,            entrarAlarma);

  stateMachine.SetOnEntering(BLOQUEO,           entrarBloqueo);


  stateMachine.SetOnLeaving(INICIO,            salirInicio);

  stateMachine.SetOnLeaving(CONFIG,            salirConfig);

  stateMachine.SetOnLeaving(MONITOR_AMBIENTAL, salirMonitorAmbiental);

  stateMachine.SetOnLeaving(MONITOR_INTRUSOS,  salirMonitorIntrusos);

  stateMachine.SetOnLeaving(ALARMA,            salirAlarma);

  stateMachine.SetOnLeaving(BLOQUEO,           salirBloqueo);

}


// ── LOGICA DE CADA ESTADO ─────────────────────────────────────────────────

void loopInicio() {

  TaskFueraFranja.Update();

  TaskClaveInc.Update();

  TaskHoraOk.Update();

  TaskHoraInv.Update();

  TaskTarjetaInv.Update();


  if (!horaConfigurada) {

    char key = keypad.getKey();

    if (!key) return;


    if (key >= '0' && key <= '9' && idxHora < 4) {

      bufHora[idxHora++] = key;

      lcd.setCursor(6 + idxHora - 1, 1);

      lcd.print(key);

    } else if (key == '#' && idxHora == 4) {

      bufHora[4] = '\0';

      byte h = (bufHora[0]-'0')*10 + (bufHora[1]-'0');

      byte m = (bufHora[2]-'0')*10 + (bufHora[3]-'0');

      if (h < 24 && m < 60) {

        horaActual      = h;

        minutoActual    = m;

        horaBase        = h;

        minBase         = m;

        horaConfigurada = true;

        tInicioSistema  = millis();

        idxHora         = 0;

        lcd.clear();

        lcd.print("Hora OK");

        TaskHoraOk.Start();

      } else {

        lcd.clear();

        lcd.print("Hora invalida");

        TaskHoraInv.Start();

      }

    } else if (key == '*') {

      idxHora = 0;

      lcd.clear();

      lcd.print("Ingrese hora:");

      lcd.setCursor(0, 1);

      lcd.print("HHMM: ");

    }

    return;

  }


  char key = keypad.getKey();


  if (key == '*') {

    idx = 0;

    memset(clave_user, 0, sizeof(clave_user));

    lcd.clear();

    lcd.print("Ingrese clave:");

    return;

  }


  if (key >= '0' && key <= '9') {

    if (idx < NUM_DIGIT) {

      clave_user[idx] = key;

      lcd.setCursor(idx, 1);

      lcd.print("*");

      idx++;

    }


    if (idx == NUM_DIGIT) {

      clave_user[NUM_DIGIT] = '\0';

      int encontrado = buscarPorClave(clave_user);

      idx = 0;

      memset(clave_user, 0, sizeof(clave_user));


      if (encontrado >= 0) {

        autenticarUsuario(encontrado);

      } else {

        cont_intentos++;

        DEBUG("Intento fallido");

        lcd.clear();

        lcd.print("Clave incorrecta");

        TaskClaveInc.Start();

      }

    }

    return;

  }


  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {

    for (byte i = 0; i < 4 && i < mfrc522.uid.size; i++) {

      uidLeido[i] = mfrc522.uid.uidByte[i];

    }

    mfrc522.PICC_HaltA();


    int encontrado = buscarPorUID(uidLeido);

    if (encontrado >= 0) {

      autenticarUsuario(encontrado);

    } else {

      lcd.clear();

      lcd.print("Tarjeta invalida");

      TaskTarjetaInv.Start();

    }

  }

}


void loopConfig() {

  TaskNoCoinciden.Update();

  TaskClaveYaUsada.Update();

  TaskClaveCambiada.Update();


  if (!puertaYaCerrada && millis() - tPuertaAbierta >= T_PUERTA_ABIERTA) {

    myservo.write(SERVO_CERRADO);

    setLED(false, false, false);

    puertaYaCerrada = true;

    lcd.clear();

    lcd.print("Puerta cerrada");

    lcd.setCursor(0, 1);

    if (cambioClaveActivo) {

      lcd.print("A:cambiar clave");

    } else {

      lcd.print("A:clave  *:salir");

    }

  }


  if (!puertaYaCerrada) return;


  char key = keypad.getKey();

  if (!key) return;


  if (key == '*') {

    inputFSM = INPUT_TIMEOUT;

    return;

  }


  if (key == 'A' || cambioClaveActivo) {

    if (faseClave == 0) {

      cambioClaveActivo = true;

      idxNueva = 0;

      lcd.clear();

      lcd.print("Nueva clave:");

      lcd.setCursor(0, 1);

      lcd.print("(4 digitos)");

      faseClave = 1;

      return;

    }

  }


  if (faseClave == 1 && key >= '0' && key <= '9') {

    if (idxNueva < NUM_DIGIT) {

      claveNueva1[idxNueva] = key;

      lcd.setCursor(idxNueva, 1);

      lcd.print("*");

      idxNueva++;

    }

    if (idxNueva == NUM_DIGIT) {

      claveNueva1[NUM_DIGIT] = '\0';

      idxNueva = 0;

      faseClave = 2;

      lcd.clear();

      lcd.print("Confirme clave:");

    }

    return;

  }


  if (faseClave == 2 && key >= '0' && key <= '9') {

    if (idxNueva < NUM_DIGIT) {

      claveNueva2[idxNueva] = key;

      lcd.setCursor(idxNueva, 1);

      lcd.print("*");

      idxNueva++;

    }

    if (idxNueva == NUM_DIGIT) {

      claveNueva2[NUM_DIGIT] = '\0';


      if (strcmp(claveNueva1, claveNueva2) != 0) {

        lcd.clear();

        lcd.print("No coinciden");

        TaskNoCoinciden.Start();

        return;

      }


      Usuario u;

      leerUsuario(usuario_actual, u);

      if (claveRepetida(u, claveNueva1)) {

        lcd.clear();

        lcd.print("Clave ya usada");

        TaskClaveYaUsada.Start();

        return;

      }


      cambiarClave(usuario_actual, claveNueva1);

      lcd.clear();

      lcd.print("Clave cambiada!");

      TaskClaveCambiada.Start();

    }

    return;

  }

}


void loopMonitorAmbiental() {

  Task1.Update();

  Task2.Update();


  static unsigned long tLCD = 0;

  if (millis() - tLCD >= 1000) {

    tLCD = millis();

    lcd.clear();

    lcd.print("AMB");


    if (horaConfigurada) {

      char buf[6];

      snprintf(buf, sizeof(buf), " %02d:%02d", horaActual, minutoActual);

      lcd.print(buf);

    }


    lcd.setCursor(0, 1);

    lcd.print("T:");

    lcd.print((int)tempLeida);

    lcd.print("C L:");

    lcd.print(luzLeida / 10);

  }


  if (tempLeida < UMBRAL_TEMP_BAJO && luzLeida < UMBRAL_LUZ_BAJO) {

    lcd.clear();

    lcd.print("ALARMA AMBIENTAL");

    inputFSM = INPUT_SENSOR_MAL;

    return;

  }


  if (millis() - tEntradaEstado >= 5000) {

    inputFSM = INPUT_TIMEOUT;

    return;

  }


  char key = keypad.getKey();

  if (key == '*' && usuario_actual >= 0) {

    inputFSM = INPUT_CLAVE_OK;

  }

}


void loopMonitorIntrusos() {

  Task3.Update();

  Task4.Update();

  TaskIntrusoDet.Update();


  static unsigned long tLCD = 0;

  if (millis() - tLCD >= 1000) {

    tLCD = millis();

    lcd.clear();

    lcd.print("INTRUS");

    lcd.setCursor(0, 1);

    lcd.print("Hall:");

    lcd.print(hallLeido > UMBRAL_HALL ? 1 : 0);

    lcd.print(" Mic:");

    lcd.print((micLeido > UMBRAL_MIC || micDig == HIGH) ? 1 : 0);

  }


  if (hallLeido > UMBRAL_HALL && (micLeido > UMBRAL_MIC || micDig == HIGH)) {

    cont_mic++;

    if (cont_mic >= UMBRAL_MIC_VECES) {

      cont_mic = 0;

      lcd.clear();

      lcd.print("INTRUSO!");

      lcd.setCursor(0, 1);

      lcd.print("Hall+Mic activos");

      TaskIntrusoDet.Start();

      return;

    }

  } else {

    if (cont_mic > 0) cont_mic--;

  }


  if (millis() - tEntradaEstado >= 2000) {

    inputFSM = INPUT_TIMEOUT;

    return;

  }


  char key = keypad.getKey();

  if (key == '#' && usuario_actual >= 0) {

    inputFSM = INPUT_CLAVE_OK;

  }

}


void loopAlarma() {

  Task3Alarmas.Update();

  parpadeoRojo(300, 700);

  digitalWrite(PIN_BUZZER, LOW);


  static unsigned long tLCD = 0;

  if (millis() - tLCD >= 500) {

    tLCD = millis();

    lcd.setCursor(0, 1);

    lcd.print("Alarmas: ");

    lcd.print(cont_alarmas);

  }


  if (cont_alarmas >= 3 && millis() - tPrimeraAlarma < T_ALARMA_MAX) {

    if (!Task3Alarmas.IsActive()) {

      lcd.clear();

      lcd.print("3 alarmas!");

      Task3Alarmas.Start();

    }

    return;

  }


  unsigned long tEspera = (estadoAntesDeAlarma == MONITOR_AMBIENTAL)

                           ? T_ALARMA_AMB : T_ALARMA_INT;


  if (millis() - tEntradaEstado >= tEspera) {

    if (estadoAntesDeAlarma == MONITOR_AMBIENTAL) {

      inputFSM = INPUT_ALARMA_AMB;

    } else {

      inputFSM = INPUT_ALARMA_INT;

    }

  }

}


void loopBloqueo() {

  parpadeoRojo(100, 500);


  static unsigned long tLCD = 0;

  if (millis() - tLCD >= 600) {

    tLCD = millis();

    lcd.setCursor(0, 1);

    lcd.print("Pres. boton     ");

  }

}


// ── SETUP ──────────────────────────────────────────────────────────────────

void setup() {

  Serial.begin(9600);


  pinMode(PIN_RED,     OUTPUT);

  pinMode(PIN_GREEN,   OUTPUT);

  pinMode(PIN_BLUE,    OUTPUT);

  pinMode(PIN_BUZZER,  OUTPUT);

  pinMode(PIN_BOTON,   INPUT_PULLUP);

  pinMode(PIN_MIC_DIG, INPUT);


  digitalWrite(PIN_BUZZER, HIGH);


  myservo.attach(PIN_SERVO);

  myservo.write(SERVO_CERRADO);


  lcd.begin(16, 2);

  lcd.print("Iniciando...");


  SPI.begin();

  mfrc522.PCD_Init();


  byte marca;

  EEPROM.get(MAX_USUARIOS * sizeof(Usuario), marca);

  if (marca != 0xAB) {

    cargarUsuariosIniciales();

    EEPROM.put(MAX_USUARIOS * sizeof(Usuario), (byte)0xAB);

  } else {

    DEBUG("Usuarios ya en EEPROM");

  }


  Serial.println("Configurando FSM...");

  setupStateMachine();

  stateMachine.SetState(INICIO, false, true);

  Serial.println("FSM lista");

}


// ── LOOP ───────────────────────────────────────────────────────────────────

void loop() {

  actualizarHora();

  inputFSM = INPUT_UNKNOWN;


  if (digitalRead(PIN_BOTON) == LOW) {

    if (millis() - tUltimoBoton > 200) {

      tUltimoBoton = millis();

      inputFSM = INPUT_RESET;

    }

  }


  int estadoActual = stateMachine.GetState();


  if (estadoActual == INICIO)            loopInicio();

  if (estadoActual == CONFIG)            loopConfig();

  if (estadoActual == MONITOR_AMBIENTAL) loopMonitorAmbiental();

  if (estadoActual == MONITOR_INTRUSOS)  loopMonitorIntrusos();

  if (estadoActual == ALARMA)            loopAlarma();

  if (estadoActual == BLOQUEO)           loopBloqueo();


  stateMachine.Update();

}

