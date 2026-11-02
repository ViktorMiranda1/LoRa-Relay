/*
 * Ping-Pong / Relay / Relay Dinamico / Troca de SF
 * Placa: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * Baseado no exemplo "pingpong" da Heltec.
 *
 * O MESMO codigo vai nos 3 radios. Mude apenas NODE_ID antes de gravar:
 *   ID 1 = radio com os botoes (origem do ping)
 *   ID 2 = par do ping-pong (responde PONG)
 *   ID 3 = relay
 *
 * Botoes (somente no ID 1), com resistor de pull-down de 10k:
 *   3V3 ---[botao]---+--- GPIO
 *                    |
 *                  [10k]
 *                    |
 *                   GND
 *   Botao solto = LOW, pressionado = HIGH.
 *
 * Controles (ID 1):
 *   ESQ ............ inicia ping-pong (depois: pausa/retoma)
 *   DIR ............ ativa modo RELAY (envia 3 avisos)
 *   DIR (em RELAY) . pede confirmacao no LCD -> DIR confirma modo DINAMICO
 *   DIR (em DIN.) .. volta ao ping-pong direto
 *   ESQ+DIR ........ menu de SF: DIR troca opcao, ESQ confirma
 *                    (ESQ+DIR de novo cancela)
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"

// ===================== ESCOLHA O ID DESTE RADIO =====================
#define NODE_ID 1
// ====================================================================

#define BTN_LEFT   6     // GPIO do botao esquerdo
#define BTN_RIGHT  7     // GPIO do botao direito

// ---------------- Parametros de radio (padrao) ----------------
#define RF_FREQUENCY               915000000 // Hz
#define TX_OUTPUT_POWER            14        // dBm
#define LORA_BANDWIDTH             0         // 0: 125 kHz
#define LORA_SF_DEFAULT            7         // SF7..SF12
#define LORA_CODINGRATE            1         // 1: 4/5
#define LORA_PREAMBLE_LENGTH       8
#define LORA_SYMBOL_TIMEOUT        0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false

// ---------------- Protocolo ----------------
#define MAGIC       0xA5
#define BCAST       0xFF
#define ID_ORIGEM   1
#define ID_DESTINO  2
#define ID_RELAY    3

#define MODE_REPEATS   3     // avisos de troca de modo
#define SF_REPEATS     5     // repeticoes do aviso de SF (redundancia)
#define PING_INTERVAL  1000  // ms entre um PONG e o proximo PING
#define REPLY_DELAY    200   // ms antes do ID 2 responder (modos direto/relay)

// Modo dinamico (anti-colisao do ACK)
#define ACK_SLOTS      6     // numero de slots de backoff aleatorio
#define ACK_PROC_MS    30    // tempo de processamento antes do 1o slot
#define DYN_MAX_TRIES  3     // tentativas da origem se nao vier ACK
#define DYN_TTL        2     // saltos maximos
#define LBT_RSSI_DBM   -85   // canal "ocupado" acima deste RSSI
#define LBT_MAX_DEFER  3     // quantas vezes adiar por canal ocupado

#define MAX_DATA   20
#define HDR_LEN    10

// ---------------- Tipos ----------------
enum PktType : uint8_t { T_PING = 1, T_PONG, T_MODE, T_SF, T_ACK };
enum Mode    : uint8_t { M_IDLE = 0, M_PINGPONG, M_RELAY, M_DYNAMIC };
enum UiState : uint8_t { UI_NORMAL, UI_CONFIRM_DYN, UI_SF_MENU };
enum BtnEvent: uint8_t { EV_LEFT, EV_RIGHT, EV_BOTH };
enum PingState : uint8_t { PS_IDLE, PS_WAIT_PONG };

const char *MODE_NAME[] = { "IDLE", "PING-PONG", "RELAY", "DINAMICO" };
const char *TYPE_NAME[] = { "?", "PING", "PONG", "MODO", "SF", "ACK" };

// Cabecalho de 10 bytes + dados (texto)
typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t type;
  uint8_t src;    // origem do pacote
  uint8_t dst;    // destino final
  uint8_t from;   // quem transmitiu este salto
  uint8_t to;     // proximo salto (0xFF = broadcast)
  uint8_t seq;
  uint8_t ttl;
  uint8_t arg;    // MODE: modo | SF: novo SF | ACK: src do pacote confirmado
  uint8_t aux;    // MODE/SF: repeticoes restantes | ACK: tipo confirmado
  char    data[MAX_DATA + 1];
} Packet;

struct RxItem { Packet p; int16_t rssi; int8_t snr; };
struct SeenE  { uint8_t src, type, seq; bool used; };

// ---------------- Globais ----------------
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
static RadioEvents_t RadioEvents;

uint8_t currentSF = LORA_SF_DEFAULT;
Mode    mode = M_IDLE;
UiState ui = UI_NORMAL;
uint8_t sfSel = LORA_SF_DEFAULT;
uint8_t mySeq;

uint32_t stTx = 0, stRx = 0, stLost = 0, stRelay = 0, stAck = 0;
int16_t  lastRssi = 0;
int8_t   lastSnr = 0;
char     lastInfo[32] = "Aguardando...";

// Fila de TX
#define TXQ_SIZE 6
Packet  txq[TXQ_SIZE];
uint8_t txHead = 0, txCount = 0;
bool    txBusy = false;
uint32_t txStart = 0;
Packet  curTx;
bool    needRx = false;

// Fila de RX
#define RXQ_SIZE 4
RxItem  rxq[RXQ_SIZE];
uint8_t rxHead = 0, rxCount = 0;

// Cache de duplicados
#define SEEN_SIZE 16
SeenE   seenTab[SEEN_SIZE];
uint8_t seenIdx = 0;

// ID 1: gerador de PING
bool      trafficOn = false;
PingState ps = PS_IDLE;
uint32_t  nextPingAt = 0, pongDeadline = 0, pingNum = 0;
uint8_t   pingSeq = 0;

// ID 2: resposta pendente
struct { bool active; uint8_t seq; uint32_t at; } reply;

// Dinamico: envio confiavel (espera ACK)
struct { bool active; bool sent; Packet p; uint8_t tries; uint32_t deadline; } dynTx;

// Dinamico: ACK agendado com backoff
struct { bool active; Packet d; uint32_t at; uint8_t defer; uint8_t slot; } ackP;

// ID 1: avisos repetidos em broadcast (modo / SF)
struct { bool active; uint8_t type, arg, left, seq; uint32_t nextAt; } ann;

// ID 2/3: troca de SF agendada
struct { bool active; uint8_t sf; uint32_t at; } sfSw;

// ====================================================================
//                          Utilitarios
// ====================================================================
bool due(uint32_t t) { return (int32_t)(millis() - t) >= 0; }

// Tempo no ar de um pacote LoRa (BW 125k, header explicito, CRC ligado)
uint32_t airtimeMs(uint8_t len, uint8_t sf) {
  float tsym = (float)(1UL << sf) / 125000.0f * 1000.0f;
  int de  = (sf >= 11) ? 1 : 0;
  int num = 8 * len - 4 * sf + 28 + 16;
  int den = 4 * (sf - 2 * de);
  int n   = (num > 0) ? ((num + den - 1) / den) * (LORA_CODINGRATE + 4) : 0;
  float t = (LORA_PREAMBLE_LENGTH + 4.25f) * tsym + (8 + n) * tsym;
  return (uint32_t)(t + 0.5f);
}

uint32_t slotMs()          { return airtimeMs(HDR_LEN + 3, currentSF) + 40; }
uint32_t ackWaitMs()       { return ACK_PROC_MS + (ACK_SLOTS + 1 + LBT_MAX_DEFER) * slotMs() + 200; }
uint32_t dynReplyDelayMs() { return ACK_PROC_MS + (ACK_SLOTS + 1) * slotMs(); }
uint32_t repeatIntervalMs(){ return airtimeMs(HDR_LEN + MAX_DATA, currentSF) + 150; }

uint32_t pongTimeoutMs() {
  uint32_t a = airtimeMs(HDR_LEN + MAX_DATA, currentSF);
  if (mode == M_DYNAMIC) return 4 * (a + ackWaitMs()) + dynReplyDelayMs() + 1000;
  if (mode == M_RELAY)   return 4 * a + REPLY_DELAY + 1500;
  return 2 * a + REPLY_DELAY + 1000;
}

uint8_t pktLen(const Packet &p) { return HDR_LEN + strnlen(p.data, MAX_DATA); }

Packet makePkt(uint8_t type, uint8_t src, uint8_t dst, uint8_t seq) {
  Packet p;
  memset(&p, 0, sizeof(p));
  p.magic = MAGIC; p.type = type; p.src = src; p.dst = dst;
  p.from = NODE_ID; p.seq = seq;
  return p;
}

bool seenBefore(uint8_t src, uint8_t type, uint8_t seq) {
  for (int i = 0; i < SEEN_SIZE; i++)
    if (seenTab[i].used && seenTab[i].src == src && seenTab[i].type == type && seenTab[i].seq == seq)
      return true;
  return false;
}

void addSeen(uint8_t src, uint8_t type, uint8_t seq) {
  seenTab[seenIdx] = { src, type, seq, true };
  seenIdx = (seenIdx + 1) % SEEN_SIZE;
}

void setMode(Mode m) {
  if (mode == m) return;
  mode = m;
  dynTx.active = false;
  ackP.active  = false;
  reply.active = false;
  ps = PS_IDLE;
  Serial.printf("[MODO] -> %s\n", MODE_NAME[m]);
}

void applySF(uint8_t sf) {
  currentSF = sf;
  Radio.Sleep();
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    sf, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0,
                    LORA_IQ_INVERSION_ON, 5000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0,
                    LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
                    LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
                    LORA_IQ_INVERSION_ON, true);
  needRx = true;
  Serial.printf("[RADIO] SF agora = %d\n", sf);
}

// ====================================================================
//                          Transmissao
// ====================================================================
void enqueueTx(const Packet &p) {
  if (txCount >= TXQ_SIZE) { Serial.println("[TX] fila cheia, pacote descartado"); return; }
  txq[(txHead + txCount) % TXQ_SIZE] = p;
  txCount++;
}

void pumpTx() {
  if (txBusy || txCount == 0) return;
  curTx = txq[txHead];
  txHead = (txHead + 1) % TXQ_SIZE;
  txCount--;
  txBusy = true;
  txStart = millis();
  Serial.printf("[TX] %s src=%d dst=%d to=%d seq=%d \"%s\"\n",
                TYPE_NAME[curTx.type], curTx.src, curTx.dst, curTx.to, curTx.seq, curTx.data);
  Radio.Send((uint8_t *)&curTx, pktLen(curTx));
  stTx++;
}

// Envio de PING/PONG conforme o modo atual
void dynSend(Packet p) {
  p.from = NODE_ID;
  p.to = BCAST;
  dynTx.active = true;
  dynTx.sent = false;
  dynTx.p = p;
  dynTx.tries = 1;
  enqueueTx(p);
}

void sendData(Packet p) {
  if (mode == M_DYNAMIC) {
    p.ttl = DYN_TTL;
    dynSend(p);
  } else {
    p.from = NODE_ID;
    // Em RELAY todos (exceto o proprio relay) mandam para o ID 3
    p.to = (mode == M_RELAY && NODE_ID != ID_RELAY) ? ID_RELAY : p.dst;
    enqueueTx(p);
  }
}

// ====================================================================
//                       Callbacks do radio
// ====================================================================
void OnTxDone(void) {
  txBusy = false;
  needRx = true;
  // Pacote dinamico saiu: comeca a contar o tempo de espera pelo ACK
  if (dynTx.active && !dynTx.sent && curTx.to == BCAST &&
      (curTx.type == T_PING || curTx.type == T_PONG) && curTx.seq == dynTx.p.seq) {
    dynTx.sent = true;
    dynTx.deadline = millis() + ackWaitMs();
  }
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("[TX] timeout");
  txBusy = false;
  needRx = true;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  needRx = true;
  if (size < HDR_LEN || size > HDR_LEN + MAX_DATA || rxCount >= RXQ_SIZE) return;
  RxItem &it = rxq[(rxHead + rxCount) % RXQ_SIZE];
  memset(&it.p, 0, sizeof(Packet));
  memcpy(&it.p, payload, size);
  it.p.data[size - HDR_LEN] = '\0';
  it.rssi = rssi;
  it.snr = snr;
  rxCount++;
}

void OnRxTimeout(void) { needRx = true; }
void OnRxError(void)   { needRx = true; }

// ====================================================================
//                          Recepcao
// ====================================================================
void deliver(const Packet &p) {
  stRx++;
  if (p.type == T_PING && NODE_ID == ID_DESTINO) {
    if (mode == M_IDLE) setMode(M_PINGPONG);
    reply.active = true;
    reply.seq = p.seq;
    reply.at = millis() + (mode == M_DYNAMIC ? dynReplyDelayMs() : REPLY_DELAY);
    snprintf(lastInfo, sizeof(lastInfo), "%s via ID%d", p.data, p.from);
  } else if (p.type == T_PONG && NODE_ID == ID_ORIGEM) {
    if (ps == PS_WAIT_PONG && p.seq == pingSeq) {
      ps = PS_IDLE;
      nextPingAt = millis() + PING_INTERVAL;
    }
    snprintf(lastInfo, sizeof(lastInfo), "PONG#%lu via ID%d", pingNum, p.from);
  }
}

// Modo dinamico: pacote em broadcast
void handleDynData(const Packet &p) {
  if (p.src == NODE_ID) return;                 // meu proprio pacote repassado
  if (NODE_ID != ID_ORIGEM && mode != M_DYNAMIC) setMode(M_DYNAMIC);

  bool forMe = (p.dst == NODE_ID);
  if (!forMe && p.ttl == 0) return;             // nao posso repassar: nao confirmo

  // Agenda o ACK num slot aleatorio (backoff). Se outro no confirmar
  // antes, o ACK e cancelado (ver onAck).
  ackP.active = true;
  ackP.d = p;
  ackP.defer = 0;
  ackP.slot = random(0, ACK_SLOTS);
  ackP.at = millis() + ACK_PROC_MS + ackP.slot * slotMs();

  if (forMe && !seenBefore(p.src, p.type, p.seq)) {
    addSeen(p.src, p.type, p.seq);
    deliver(p);
  }
}

void onAck(const Packet &a) {
  // Supressao: alguem ja confirmou o pacote que eu ia confirmar
  if (ackP.active && a.to == ackP.d.from && a.seq == ackP.d.seq && a.arg == ackP.d.src) {
    ackP.active = false;
    Serial.printf("[ACK] ID%d confirmou primeiro, cancelando o meu\n", a.src);
  }
  // ACK para mim
  if (a.to == NODE_ID && dynTx.active && a.seq == dynTx.p.seq && a.arg == dynTx.p.src) {
    dynTx.active = false;
    snprintf(lastInfo, sizeof(lastInfo), "%s: ACK de ID%d", TYPE_NAME[dynTx.p.type], a.src);
    Serial.printf("[ACK] recebido de ID%d\n", a.src);
  }
}

void onSfMsg(const Packet &p) {
  if (p.arg < 7 || p.arg > 12) return;
  if (p.arg == currentSF) {           // mesmo SF: so volta ao ping-pong
    setMode(M_PINGPONG);
    return;
  }
  if (!sfSw.active) {
    sfSw.active = true;
    sfSw.sf = p.arg;
    // espera as repeticoes restantes terminarem antes de trocar
    sfSw.at = millis() + p.aux * repeatIntervalMs() + 300;
    snprintf(lastInfo, sizeof(lastInfo), "Aviso: mudar p/ SF%d", p.arg);
  }
}

void handlePacket(Packet &p, int16_t rssi, int8_t snr) {
  if (p.magic != MAGIC || p.from == NODE_ID) return;
  lastRssi = rssi;
  lastSnr = snr;
  Serial.printf("[RX] %s src=%d dst=%d from=%d to=%d seq=%d \"%s\" RSSI=%d SNR=%d\n",
                p.type <= T_ACK ? TYPE_NAME[p.type] : "?", p.src, p.dst, p.from, p.to,
                p.seq, p.data, rssi, snr);

  switch (p.type) {
    case T_MODE:
      if (NODE_ID != ID_ORIGEM && p.arg <= M_DYNAMIC) {
        setMode((Mode)p.arg);
        snprintf(lastInfo, sizeof(lastInfo), "Aviso modo %s", MODE_NAME[p.arg]);
      }
      return;
    case T_SF:
      if (NODE_ID != ID_ORIGEM) onSfMsg(p);
      return;
    case T_ACK:
      onAck(p);
      return;
    case T_PING:
    case T_PONG:
      break;
    default:
      return;
  }

  if (p.to == BCAST) { handleDynData(p); return; }   // modo dinamico
  if (p.to != NODE_ID) return;                       // salto nao e para mim

  if (p.dst != NODE_ID) {                            // sou o relay
    if (NODE_ID == ID_RELAY) {
      Packet f = p;
      f.from = NODE_ID;
      f.to = p.dst;
      enqueueTx(f);
      stRelay++;
      snprintf(lastInfo, sizeof(lastInfo), "Repassei %s", p.data);
    }
    return;
  }
  deliver(p);
}

// ====================================================================
//                       Tarefas do loop
// ====================================================================
void runReply() {
  if (!reply.active || !due(reply.at)) return;
  reply.active = false;
  Packet p = makePkt(T_PONG, NODE_ID, ID_ORIGEM, reply.seq);
  snprintf(p.data, sizeof(p.data), "PONG rssi %d", lastRssi);
  sendData(p);
}

void runAck() {
  if (!ackP.active || !due(ackP.at) || txBusy) return;
  // Escuta antes de falar: canal ocupado -> adia um slot
  if (Radio.Rssi(MODEM_LORA) > LBT_RSSI_DBM && ackP.defer < LBT_MAX_DEFER) {
    ackP.defer++;
    ackP.at = millis() + slotMs();
    return;
  }
  ackP.active = false;

  Packet a = makePkt(T_ACK, NODE_ID, ackP.d.from, ackP.d.seq);
  a.to = ackP.d.from;
  a.arg = ackP.d.src;
  a.aux = ackP.d.type;
  strcpy(a.data, "ACK");
  enqueueTx(a);
  stAck++;
  Serial.printf("[ACK] enviando para ID%d (slot %d)\n", a.to, ackP.slot);

  // Confirmei um pacote que nao e meu: agora sou responsavel por repassa-lo
  const Packet &d = ackP.d;
  if (d.dst != NODE_ID && d.ttl > 0 && !seenBefore(d.src, d.type, d.seq)) {
    addSeen(d.src, d.type, d.seq);
    Packet f = d;
    f.ttl--;
    dynSend(f);
    stRelay++;
    snprintf(lastInfo, sizeof(lastInfo), "Repassei %s", d.data);
  }
}

void runDynTx() {
  if (!dynTx.active || !dynTx.sent || !due(dynTx.deadline)) return;
  if (dynTx.tries < DYN_MAX_TRIES) {
    dynTx.tries++;
    dynTx.sent = false;
    Serial.printf("[DIN] sem ACK, tentativa %d\n", dynTx.tries);
    enqueueTx(dynTx.p);
  } else {
    dynTx.active = false;
    snprintf(lastInfo, sizeof(lastInfo), "Sem ACK (%d tent.)", DYN_MAX_TRIES);
  }
}

void runSfSwitch() {
  if (!sfSw.active || !due(sfSw.at) || txBusy) return;
  sfSw.active = false;
  txCount = 0;
  applySF(sfSw.sf);
  setMode(M_PINGPONG);
  snprintf(lastInfo, sizeof(lastInfo), "Trocado p/ SF%d", currentSF);
}

// ---------------- Somente ID 1 ----------------
void startAnnounce(uint8_t type, uint8_t arg, uint8_t count) {
  ann.active = true;
  ann.type = type;
  ann.arg = arg;
  ann.left = count;
  ann.seq = mySeq++;
  ann.nextAt = millis() + 100;
  ps = PS_IDLE;
  dynTx.active = false;
}

void runAnnouncer() {
  if (!ann.active || !due(ann.nextAt) || txBusy || txCount) return;
  if (ann.left > 0) {
    Packet p = makePkt(ann.type, NODE_ID, BCAST, ann.seq);
    p.to = BCAST;
    p.arg = ann.arg;
    p.aux = ann.left - 1;                // quantas repeticoes ainda virao
    if (ann.type == T_SF) snprintf(p.data, sizeof(p.data), "SF%d", ann.arg);
    else                  snprintf(p.data, sizeof(p.data), "%s", MODE_NAME[ann.arg]);
    enqueueTx(p);
    ann.left--;
    ann.nextAt = millis() + repeatIntervalMs();
    return;
  }
  // Todas as repeticoes enviadas
  ann.active = false;
  if (ann.type == T_SF) {
    applySF(ann.arg);
    setMode(M_PINGPONG);
    snprintf(lastInfo, sizeof(lastInfo), "Trocado p/ SF%d", currentSF);
  }
  trafficOn = true;
  ps = PS_IDLE;
  nextPingAt = millis() + (ann.type == T_SF ? 1500 : 500);
}

void runPinger() {
  if (ps == PS_WAIT_PONG && due(pongDeadline)) {
    stLost++;
    ps = PS_IDLE;
    nextPingAt = millis() + 200;
    snprintf(lastInfo, sizeof(lastInfo), "PING#%lu perdido", pingNum);
  }
  if (!trafficOn || ann.active || ui != UI_NORMAL || mode == M_IDLE) return;
  if (ps != PS_IDLE || !due(nextPingAt) || txBusy || txCount || dynTx.active) return;

  pingSeq = mySeq++;
  pingNum++;
  Packet p = makePkt(T_PING, NODE_ID, ID_DESTINO, pingSeq);
  snprintf(p.data, sizeof(p.data), "PING %lu", pingNum);
  sendData(p);
  ps = PS_WAIT_PONG;
  pongDeadline = millis() + pongTimeoutMs();
}

void onButton(BtnEvent ev) {
  Serial.printf("[BTN] %s\n", ev == EV_LEFT ? "ESQ" : ev == EV_RIGHT ? "DIR" : "AMBOS");

  if (ui == UI_SF_MENU) {
    if (ev == EV_RIGHT) {
      sfSel = (sfSel >= 12) ? 7 : sfSel + 1;
    } else if (ev == EV_LEFT) {
      ui = UI_NORMAL;
      startAnnounce(T_SF, sfSel, SF_REPEATS);
      snprintf(lastInfo, sizeof(lastInfo), "Enviando SF%d", sfSel);
    } else {
      ui = UI_NORMAL;                    // ambos = cancelar
    }
    return;
  }

  if (ui == UI_CONFIRM_DYN) {
    ui = UI_NORMAL;
    if (ev == EV_RIGHT) {
      setMode(M_DYNAMIC);
      startAnnounce(T_MODE, M_DYNAMIC, MODE_REPEATS);
    } else {
      snprintf(lastInfo, sizeof(lastInfo), "Dinamico cancelado");
    }
    return;
  }

  switch (ev) {
    case EV_BOTH:
      ui = UI_SF_MENU;
      sfSel = currentSF;
      break;
    case EV_LEFT:
      if (mode == M_IDLE) {
        setMode(M_PINGPONG);
        trafficOn = true;
        nextPingAt = millis();
      } else {
        trafficOn = !trafficOn;
      }
      break;
    case EV_RIGHT:
      if (mode == M_IDLE || mode == M_PINGPONG) {
        setMode(M_RELAY);
        startAnnounce(T_MODE, M_RELAY, MODE_REPEATS);
      } else if (mode == M_RELAY) {
        ui = UI_CONFIRM_DYN;
      } else {
        setMode(M_PINGPONG);
        startAnnounce(T_MODE, M_PINGPONG, MODE_REPEATS);
      }
      break;
  }
}

// Evento disparado ao SOLTAR os botoes; se os dois estiveram
// pressionados juntos em algum momento, vira EV_BOTH.
void pollButtons() {
  static uint32_t lastSample = 0, releasedAt = 0;
  static bool active = false, sawL = false, sawR = false, sawBoth = false;
  if (millis() - lastSample < 10) return;
  lastSample = millis();

  bool l = digitalRead(BTN_LEFT) == HIGH;
  bool r = digitalRead(BTN_RIGHT) == HIGH;
  if (l || r) {
    active = true;
    sawL |= l;
    sawR |= r;
    if (l && r) sawBoth = true;
    releasedAt = millis();
    return;
  }
  if (!active || millis() - releasedAt < 60) return;   // debounce na soltura

  BtnEvent ev = (sawBoth || (sawL && sawR)) ? EV_BOTH : (sawL ? EV_LEFT : EV_RIGHT);
  active = sawL = sawR = sawBoth = false;
  onButton(ev);
}

// ====================================================================
//                           Display
// ====================================================================
void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void updateDisplay() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();

  char l[40];
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  if (ui == UI_SF_MENU) {
    display.drawString(0, 0, "TROCAR SF (todos)");
    snprintf(l, sizeof(l), "Atual: SF%d", currentSF);
    display.drawString(0, 12, l);
    display.setFont(ArialMT_Plain_16);
    snprintf(l, sizeof(l), "> SF%d <", sfSel);
    display.drawString(34, 24, l);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 42, "DIR: proxima opcao");
    display.drawString(0, 52, "ESQ: confirmar");
  } else if (ui == UI_CONFIRM_DYN) {
    display.drawString(0, 0, "Ativar modo");
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 12, "RELAY DINAMICO?");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 34, "Broadcast + ACK");
    display.drawString(0, 46, "DIR: confirmar  ESQ: nao");
  } else {
    snprintf(l, sizeof(l), "ID%d  SF%d  %s", NODE_ID, currentSF, MODE_NAME[mode]);
    display.drawString(0, 0, l);

    if (NODE_ID == ID_ORIGEM)
      snprintf(l, sizeof(l), "TX:%lu RX:%lu Perd:%lu", stTx, stRx, stLost);
    else if (NODE_ID == ID_RELAY)
      snprintf(l, sizeof(l), "Repass:%lu ACK:%lu", stRelay, stAck);
    else
      snprintf(l, sizeof(l), "RX:%lu TX:%lu ACK:%lu", stRx, stTx, stAck);
    display.drawString(0, 12, l);

    snprintf(l, sizeof(l), "RSSI:%d SNR:%d", lastRssi, lastSnr);
    display.drawString(0, 24, l);
    display.drawString(0, 36, lastInfo);

    if (ann.active)
      snprintf(l, sizeof(l), "Anunciando... (%d)", ann.left);
    else if (sfSw.active)
      snprintf(l, sizeof(l), "Trocando p/ SF%d...", sfSw.sf);
    else if (NODE_ID == ID_ORIGEM)
      snprintf(l, sizeof(l), trafficOn ? "ESQ:pausa DIR:modo" : "ESQ:iniciar DIR:modo");
    else
      l[0] = '\0';
    display.drawString(0, 50, l);
  }
  display.display();
}

// ====================================================================
//                         Setup / Loop
// ====================================================================
void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  VextON();
  delay(100);
  display.init();
  display.setFont(ArialMT_Plain_10);

#if NODE_ID == ID_ORIGEM
  pinMode(BTN_LEFT, INPUT);    // pull-down externo de 10k
  pinMode(BTN_RIGHT, INPUT);
#endif

  mySeq = (uint8_t)esp_random();

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError   = OnRxError;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  applySF(LORA_SF_DEFAULT);

  Serial.printf("No ID%d pronto. 915 MHz, SF%d, BW125, CR4/5, 14 dBm\n", NODE_ID, currentSF);
}

void loop() {
  Radio.IrqProcess();

  if (needRx && !txBusy) {
    needRx = false;
    Radio.Rx(0);
  }

  while (rxCount) {
    RxItem it = rxq[rxHead];
    rxHead = (rxHead + 1) % RXQ_SIZE;
    rxCount--;
    handlePacket(it.p, it.rssi, it.snr);
  }

#if NODE_ID == ID_ORIGEM
  pollButtons();
  runAnnouncer();
  runPinger();
#endif
  runReply();
  runAck();
  runDynTx();
  runSfSwitch();
  pumpTx();

  // Seguranca: se o TxDone nunca chegar, libera o radio
  if (txBusy && millis() - txStart > airtimeMs(pktLen(curTx), currentSF) + 3000) {
    txBusy = false;
    Radio.Sleep();
    needRx = true;
  }

  updateDisplay();
}
