#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define TELIT_UART_NODE  DT_NODELABEL(uart21)
#define BUF_SIZE         256
#define DTR_PIN          11      /* P1.11 -> EVB GPIO01 -> module C108/DTR */
#define NB_SCANS         3       /* nombre de scans WiFi par cycle         */
#define SLEEP_MS         300000  /* durée de veille : 5 minutes            */
#define SLEEP_MIN        (SLEEP_MS / 60000)
#define GNSS_TIMEOUT_MS      120000  /* 2 min max pour le premier fix GNSS     */
#define GNSS_REFIX_TIMEOUT_MS 60000  /* 1 min max pour un re-fix après arrêt   */
#define GNSS_FRAMES_PER_SEND      5  /* trames valides avant chaque envoi      */
#define GNSS_SEND_COUNT           5  /* nombre d'envois GNSS par cycle         */

#define MQTT_BROKER      "c6f757b2957040889bf448086d07d8a2.s1.eu.hivemq.cloud"
#define MQTT_PORT        8883
#define MQTT_CLIENT_ID   "telit-tracker-001"
#define MQTT_USER        "LBLABOURGADE"
#define MQTT_PASS        "d3lc2rr0@4M33!"
#define MQTT_TOPIC_WIFI  "tracker/wifi"
#define MQTT_TOPIC_GNSS  "tracker/gnss"
#define MQTT_BUF_SIZE    512

K_MSGQ_DEFINE(telit_msgq, BUF_SIZE, 16, 4);

static const struct device *telit_uart = DEVICE_DT_GET(TELIT_UART_NODE);
static const struct device *gpio1_dev  = DEVICE_DT_GET(DT_NODELABEL(gpio1));

static char rx_buf[BUF_SIZE];
static int  rx_buf_pos;

static char mqtt_payload[MQTT_BUF_SIZE];
static char gnss_payload[64];

/* ── ISR réception UART : découpe les lignes et les met en file ───── */
static void telit_rx_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev))   return;
	if (!uart_irq_rx_ready(dev)) return;

	while (uart_fifo_read(dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			rx_buf[rx_buf_pos] = '\0';
			k_msgq_put(&telit_msgq, rx_buf, K_NO_WAIT);
			rx_buf_pos = 0;
		} else if (c != '\n' && c != '\r' && rx_buf_pos < BUF_SIZE - 1) {
			rx_buf[rx_buf_pos++] = c;
		}
	}
}

/* ── Envoi d'une commande AT (polling TX + CR) ────────────────────── */
static void send_at(const char *cmd)
{
	printk("> %s\n", cmd);
	for (int i = 0; cmd[i]; i++)
		uart_poll_out(telit_uart, cmd[i]);
	uart_poll_out(telit_uart, '\r');
}

/* ── Affichage d'une ligne de réponse, WiFi en cyan, NMEA en vert ─── */
static void print_line(const char *line)
{
	if (strlen(line) < 5) {
		printk("[WIFI] %s\033[0m\n", line);
	} else if (line[0] == '$') {
		printk("[GNSS] %s\033[0m\n", line);
	} else {
		printk("< %s\n", line);
	}
}

/* ── Attend une ligne contenant 'expected', timeout en ms ────────── */
static int wait_for(const char *expected, int timeout_ms)
{
	char line[BUF_SIZE];
	int64_t end = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < end) {
		if (k_msgq_get(&telit_msgq, line, K_MSEC(100)) == 0) {
			print_line(line);
			if (expected && strstr(line, expected)) {
				return 0;
			}
		}
	}
	return -ETIMEDOUT;
}

/* ── Vide la file, affiche et capture les lignes %WIFICMD dans mqtt_payload ── */
static void drain_queue_capture(int extra_ms)
{
	char line[BUF_SIZE];
	int pos = 0;

	mqtt_payload[0] = '\0';
	k_msleep(extra_ms);

	while (k_msgq_get(&telit_msgq, line, K_NO_WAIT) == 0) {
		print_line(line);
		const char *w = strstr(line, "%WIFICMD: ");
		if (!w || pos >= MQTT_BUF_SIZE - 80) continue;
		w += 10;

		char mac[20] = {0}, ssid[33] = "(hidden)";
		int rssi = 0, ch = 0;

		if (*w=='"'){w++;int i=0;while(*w&&*w!='"'&&i<18)mac[i++]=*w++;if(*w)w++;}
		if (*w==','){w++;rssi=atoi(w);while(*w&&*w!=',')w++;}
		if (*w==','){w++;while(*w&&*w!=',')w++;}
		if (*w==','){w++;ch=atoi(w);while(*w&&*w!=',')w++;}
		if (*w==','){w++;if(*w=='"'){w++;int i=0;while(*w&&*w!='"'&&i<32)ssid[i++]=*w++;ssid[i]='\0';}}
		if (ssid[0]=='\0') strncpy(ssid,"(hidden)",32);

		if (pos > 0) pos += snprintf(mqtt_payload+pos, MQTT_BUF_SIZE-pos, " // ");
		pos += snprintf(mqtt_payload+pos, MQTT_BUF_SIZE-pos,
			       "%s  %ddBm  ch%d  %s", ssid, rssi, ch, mac);
	}

	if (pos > 0) mqtt_payload[pos - 1] = '\0';
	else         mqtt_payload[0] = '\0';
}

/* ── Vide toutes les lignes en attente (sans capture) ─────────────── */
static void drain_queue(int extra_ms)
{
	char line[BUF_SIZE];

	k_msleep(extra_ms);
	while (k_msgq_get(&telit_msgq, line, K_NO_WAIT) == 0) {
		print_line(line);
	}
}

/* ── Contrôle DTR : HIGH = module actif, LOW = module en veille ───── */
static void dtr_set(int val)
{
	gpio_pin_set(gpio1_dev, DTR_PIN, !val); /* actif bas : réveil=LOW, veille=HIGH */
	printk("[DTR] %s\n", val ? "HIGH (actif)" : "LOW  (veille)");
}

/* ── Lit l'heure courante et affiche l'heure du prochain scan ─────── */
static void print_next_scan_time(void)
{
	char line[BUF_SIZE];
	int64_t end = k_uptime_get() + 3000;
	int hh = -1, mm = -1;

	send_at("AT+CCLK?");
	while (k_uptime_get() < end) {
		if (k_msgq_get(&telit_msgq, line, K_MSEC(100)) == 0) {
			char *comma = strchr(line, ',');
			if (comma && sscanf(comma + 1, "%d:%d", &hh, &mm) == 2 && hh >= 0) {
				mm += SLEEP_MIN;
				hh  = (hh + mm / 60) % 24;
				mm %= 60;
				printk(">>> Prochain scan vers %02d:%02d\n", hh, mm);
			} else {
				print_line(line);
			}
		}
	}
}

/* ── $GPRMC/$GNRMC avec statut A = fix valide ───────────────────────── */
static int is_gnss_fix(const char *line)
{
	if (strncmp(line, "$GPRMC,", 7) != 0 &&
	    strncmp(line, "$GNRMC,", 7) != 0)
		return 0;
	const char *comma = strchr(line + 7, ',');
	return comma && *(comma + 1) == 'A';
}

/* ── Retourne le Nième champ d'une trame NMEA (séparateur ',') ───────── */
static const char *nmea_field(const char *s, int n)
{
	for (int i = 0; i < n; i++) {
		s = strchr(s, ',');
		if (!s) return NULL;
		s++;
	}
	return s;
}

/* ── Parse $GPRMC/$GNRMC et remplit gnss_payload en degrés décimaux ──── */
/* ── Parse $GPRMC/$GNRMC et remplit gnss_payload en degrés décimaux ──── */
static void gnss_build_payload(const char *line)
{
	const char *f3 = nmea_field(line, 3);
	const char *f4 = nmea_field(line, 4);
	const char *f5 = nmea_field(line, 5);
	const char *f6 = nmea_field(line, 6);

	if (!f3 || !f4 || !f5 || !f6 || *f3 == ',' || *f5 == ',') return;

	int lat_dd = 0, lat_mm = 0, lat_frac = 0, lat_frac_digits = 0;
	int lon_ddd = 0, lon_mm = 0, lon_frac = 0, lon_frac_digits = 0;

	/* --- latitude DDMM.MMMM --- */
	const char *p = f3;
	while (*p >= '0' && *p <= '9') lat_dd = lat_dd * 10 + (*p++ - '0');
	lat_mm = lat_dd % 100;
	lat_dd = lat_dd / 100;
	if (*p == '.') {
		p++;
		while (*p >= '0' && *p <= '9') {
			lat_frac = lat_frac * 10 + (*p++ - '0');
			lat_frac_digits++;
		}
	}

	/* --- longitude DDDMM.MMMM --- */
	p = f5;
	while (*p >= '0' && *p <= '9') lon_ddd = lon_ddd * 10 + (*p++ - '0');
	lon_mm  = lon_ddd % 100;
	lon_ddd = lon_ddd / 100;
	if (*p == '.') {
		p++;
		while (*p >= '0' && *p <= '9') {
			lon_frac = lon_frac * 10 + (*p++ - '0');
			lon_frac_digits++;
		}
	}

	/* --- conversion en nanodegrés (int64) pour éviter tout débordement --- */
	/* minutes décimales = mm + frac/10^digits                               */
	/* degrés décimaux   = dd + minutes_decimales / 60                       */
	/* on travaille en unités de 1e-9 degré (nanodegrés)                    */

	int64_t scale = 1;
	for (int i = 0; i < lat_frac_digits; i++) scale *= 10;
	/* lat en nanodegrés = (lat_dd + (lat_mm + lat_frac/scale) / 60) * 1e9  */
	int64_t lat_nano = (int64_t)lat_dd * 1000000000LL
			 + ((int64_t)lat_mm * 1000000000LL
			 + (int64_t)lat_frac * (1000000000LL / scale)) / 60;
	if (*f4 == 'S') lat_nano = -lat_nano;

	scale = 1;
	for (int i = 0; i < lon_frac_digits; i++) scale *= 10;
	int64_t lon_nano = (int64_t)lon_ddd * 1000000000LL
			 + ((int64_t)lon_mm * 1000000000LL
			 + (int64_t)lon_frac * (1000000000LL / scale)) / 60;
	if (*f6 == 'W') lon_nano = -lon_nano;

	/* --- formatage : on affiche 7 décimales (précision ~1cm) --- */
	int lat_neg = lat_nano < 0; if (lat_neg) lat_nano = -lat_nano;
	int lon_neg = lon_nano < 0; if (lon_neg) lon_nano = -lon_nano;

	int lat_int = (int)(lat_nano / 1000000000LL);
	int lat_dec = (int)(lat_nano % 1000000000LL / 100); /* 7 décimales */
	int lon_int = (int)(lon_nano / 1000000000LL);
	int lon_dec = (int)(lon_nano % 1000000000LL / 100);

	snprintf(gnss_payload, sizeof(gnss_payload),
		 "%s%d.%07d,%s%d.%07d",
		 lat_neg ? "-" : "", lat_int, lat_dec,
		 lon_neg ? "-" : "", lon_int, lon_dec);
}

/* ── Détecdte +CEREG: n,stat (stat 1=home, 5=roaming) ───────────────── */
static int cereg_is_registered(const char *line)
{
	const char *p = strstr(line, "+CEREG:");
	if (!p) return 0;
	p += 7;
	while (*p == ' ') p++;
	const char *comma = strchr(p, ',');
	if (!comma) return *p == '1' || *p == '5';
	return *(comma + 1) == '1' || *(comma + 1) == '5';
}

/* ── Attend le ré-enregistrement LTE après retour CFUN=1 ────────────── */
static void wait_lte_registered(int timeout_ms)
{
	char line[BUF_SIZE];
	int64_t deadline = k_uptime_get() + timeout_ms;
	int idle_count = 0; /* CEREG=0,0 consécutifs : module détaché */

	printk("[LTE] Attente re-enregistrement...\n");
	while (k_uptime_get() < deadline) {
		int got_idle = 1; /* présumé idle jusqu'à preuve du contraire */

		send_at("AT+CEREG?");
		int64_t resp_end = k_uptime_get() + 3000;
		while (k_uptime_get() < resp_end) {
			if (k_msgq_get(&telit_msgq, line, K_MSEC(200)) == 0) {
				print_line(line);
				if (cereg_is_registered(line)) {
					printk("[LTE] Re-enregistre\n");
					drain_queue(500);
					return;
				}
				/* stat=2 (recherche) ou stat=3 (enregistrement refusé) :
				 * le module tente activement, pas idle */
				if (strstr(line, "+CEREG:") &&
				    !strstr(line, ": 0,0") && !strstr(line, ",0\r"))
					got_idle = 0;
				if (strstr(line, "OK")) break;
			}
		}

		if (got_idle) {
			idle_count++;
			if (idle_count >= 3) {
				/* Module détaché (CEREG=0,0 x3) : forcer ré-attachement */
				printk("[LTE] Module detache, ré-attachement force...\n");
				send_at("AT+CFUN=0");
				wait_for("OK", 10000);
				k_msleep(2000);
				send_at("AT+CFUN=1");
				wait_for("OK", 10000);
				idle_count = 0;
			}
		} else {
			idle_count = 0;
		}

		k_msleep(3000);
	}
	printk("[LTE] Timeout re-enregistrement\n");
}

static int  mqtt_init(void);
static int  mqtt_connect(void);
static void mqtt_publish_gnss(void);
static void mqtt_disconnect(void);

/* ── Retour LTE après une session GNSS (CFUN=4 → CFUN=1 → LTE → CFUN=5) ── */
static void gnss_restore_lte(void)
{
	send_at("AT+CFUN=1");
	wait_for("OK", 10000);
	wait_lte_registered(60000);

	/* CFUN=4 désactive le contexte PDP — le ré-activer si besoin. */
	send_at("AT#SGACT?");
	if (wait_for("#SGACT: 1,1", 3000) != 0) {
		send_at("AT#SGACT=1,1");
		wait_for("#SGACT:", 30000);
		wait_for("OK", 5000);
	} else {
		drain_queue(200);
	}

	/* Si CFUN=4 a effacé la config MQTT (broker vide), refaire mqtt_init.
	 * Si la config est déjà là, ne pas toucher : MQCONN marchera direct
	 * et tenter de reconfigurer SSL casserait la liaison existante. */
	{
		char cfg_line[BUF_SIZE];
		int  broker_set = 0;

		send_at("AT#MQCFG?");
		int64_t t = k_uptime_get() + 3000;
		while (k_uptime_get() < t) {
			if (k_msgq_get(&telit_msgq, cfg_line, K_MSEC(200)) == 0) {
				print_line(cfg_line);
				if (strstr(cfg_line, "#MQCFG: 1,") &&
				    !strstr(cfg_line, "1,\"\""))
					broker_set = 1;
				if (strstr(cfg_line, "OK")) break;
			}
		}
		if (!broker_set)
			mqtt_init();
	}

	send_at("AT+CFUN=5");
	wait_for("OK", 5000);
}

/* ── Tracking GNSS : GNSS_SEND_COUNT envois, chacun après GNSS_FRAMES_PER_SEND trames valides ── */
static void gnss_tracking_cycle(void)
{
	char line[BUF_SIZE];
	int  publish_count = 0;

	gnss_payload[0] = '\0';
	printk("=== Tracking GNSS : %d envois x %d trames ===\n",
	       GNSS_SEND_COUNT, GNSS_FRAMES_PER_SEND);

	while (publish_count < GNSS_SEND_COUNT) {
		int valid_count = 0;
		int timeout_ms = (publish_count == 0) ? GNSS_TIMEOUT_MS
						       : GNSS_REFIX_TIMEOUT_MS;

		/* --- Passe en mode GNSS (radio off) --- */
		send_at("AT+CFUN=4");
		wait_for("OK", 10000);
		k_msleep(1000); /* stabilisation avant GPSP */

		send_at("AT$GPSP=0"); /* arrêt préventif si GNSS déjà actif */
		wait_for("OK", 3000);

		send_at("AT$GPSP=1");
		if (wait_for("OK", 10000) != 0) {
			printk("[GNSS] Echec demarrage GNSS\n");
			gnss_restore_lte();
			return;
		}

		send_at("AT$GNSSNMEA=1,1");
		wait_for("OK", 3000);

		/* --- Accumule GNSS_FRAMES_PER_SEND trames valides --- */
		{
			int64_t end = k_uptime_get() + timeout_ms;
			while (k_uptime_get() < end) {
				if (k_msgq_get(&telit_msgq, line, K_MSEC(200)) == 0) {
					print_line(line);
					if (is_gnss_fix(line)) {
						gnss_build_payload(line);
						valid_count++;
						printk("[GNSS] Trame %d/%d : %s\n",
						       valid_count,
						       GNSS_FRAMES_PER_SEND,
						       gnss_payload);
						if (valid_count >= GNSS_FRAMES_PER_SEND)
							break;
					}
				}
			}
		}

		/* --- Arrêt GNSS --- */
		send_at("AT$GNSSNMEA=0,0");
		wait_for("OK", 3000);
		send_at("AT$GPSP=0");
		wait_for("OK", 5000);

		/* --- Retour LTE --- */
		gnss_restore_lte();

		if (valid_count < GNSS_FRAMES_PER_SEND) {
			printk("[GNSS] Seulement %d/%d trames, fin tracking\n",
			       valid_count, GNSS_FRAMES_PER_SEND);
			return;
		}

		/* --- Envoi MQTT --- */
		if (mqtt_connect() == 0) {
			mqtt_publish_gnss();
			mqtt_disconnect();
			publish_count++;
			printk("[GNSS] Envoi %d/%d OK\n", publish_count, GNSS_SEND_COUNT);
		}
	}

	printk("[GNSS] Tracking termine : %d positions envoyees\n", GNSS_SEND_COUNT);
}

/* ── Initialihsation WiFi (à appeler au boot ET après chaque réveil DTR) ── */
static void wifi_init(void)
{
	send_at("AT%WIFICFG=\"SET\",\"CHANNEL\",1,6,11");
	wait_for("OK", 3000);
	send_at("AT%WIFICFG=\"SET\",\"TIMEOUT\",120,2000");
	wait_for("OK", 3000);
	send_at("AT%WIFICFG=\"SET\",\"SCANTABLE\",20,0");
	wait_for("OK", 3000);
	send_at("AT%WIFIEV=\"REGSCAN\",1");
	wait_for("OK", 3000);
}

/* ── Un cycle de scan WiFi complet (NB_SCANS passages) ────────────── */
static void wifi_scan_cycle(void)
{
	printk("=== Cycle WiFi : %d scans ===\n", NB_SCANS);

	send_at("AT%WIFIEV=\"REGSCAN\",1");
	wait_for("OK", 3000);

	send_at("AT%WIFICMD=\"CLEARRES\"");
	wait_for("OK", 3000);

	for (int i = 0; i < NB_SCANS; i++) {
		printk("--- Scan %d/%d ---\n", i + 1, NB_SCANS);
		send_at("AT%WIFICMD=\"START\",0");
		if (wait_for("WIFIEVU", 5000) != 0) {
			printk("Attention : WIFIEVU non reçu pour scan %d\n", i + 1);
		}
	}

	send_at("AT%WIFICMD=\"GETRES\"");
	drain_queue_capture(1000); /* capture les résultats pour MQTT */

	send_at("AT%WIFICMD=\"CLEARRES\"");
	wait_for("OK", 2000);
}

/* ── Initialisation MQTT+SSL — appelée UNE SEULE FOIS au boot ───────── */
static int mqtt_init(void)
{
	/* --- Diagnostic : état actuel du module avant toute modification --- */
	printk("[DIAG] === Etat module avant init ===\n");
	send_at("AT#MQEN?");
	wait_for("OK", 2000);
	send_at("AT#MQCFG?");
	wait_for("OK", 2000);
	send_at("AT#MQCONN?");
	wait_for("OK", 2000);

	printk("[MQTT] Init -> %s:%d\n", MQTT_BROKER, MQTT_PORT);

	/* 1. Déconnecter et désactiver MQTT pour libérer toute liaison SSL */
	send_at("AT#MQDISC=1");
	wait_for("OK", 3000);
	send_at("AT#MQEN=1,0");
	wait_for("OK", 3000);

	/* 2. Activer SSL, le configurer, puis le DÉSACTIVER avant MQCFG.
	 *    MQCFG échoue avec "SSL already activated" si le socket est actif
	 *    quand il tente de le lier en interne. On configure pendant qu'il
	 *    est actif (requis par la doc), puis on l'éteint avant MQCFG. */
	send_at("AT#SSLEN=1,1");
	wait_for("OK", 3000);
	send_at("AT#SSLCFG=1,1,0,90,100,50");
	wait_for("OK", 3000);
	send_at("AT#SSLSECCFG=1,0,0");
	wait_for("OK", 3000);
	send_at("AT#SSLSECCFG2=1,3,1");
	wait_for("OK", 3000);
	send_at("AT#SSLEN=1,0");   /* ← désactivé : MQCFG peut lier sans conflit */
	wait_for("OK", 3000);

	/* 3. Activer MQTT (état frais, SSL non encore lié) */
	send_at("AT#MQEN=1,1");
	if (wait_for("OK", 5000) != 0) {
		printk("[MQTT] Echec MQEN\n");
		return -1;
	}

	/* 4. Configurer MQTT+SSL (socket SSL désactivé → pas de conflit) */
	{
		char cfg_cmd[100];
		snprintf(cfg_cmd, sizeof(cfg_cmd),
			 "AT#MQCFG=1,\"%s\",%d,1,1,1", MQTT_BROKER, MQTT_PORT);
		send_at(cfg_cmd);
	}
	if (wait_for("OK", 5000) != 0) {
		printk("[MQTT] Echec MQCFG\n");
		return -1;
	}

	/* 5. Réactiver SSL pour l'utilisation réelle */
	send_at("AT#SSLEN=1,1");
	wait_for("OK", 3000);

	/* --- Diagnostic : état après init --- */
	printk("[DIAG] === Etat module apres init ===\n");
	send_at("AT#MQEN?");
	wait_for("OK", 2000);
	send_at("AT#MQCFG?");
	wait_for("OK", 2000);

	printk("[MQTT] Initialise\n");
	return 0;
}

/* ── Connexion MQTT — appelée à chaque cycle (config déjà en place) ─── */
static int mqtt_connect(void)
{
	printk("=== Connexion MQTT -> %s:%d ===\n", MQTT_BROKER, MQTT_PORT);

	/* Remet le client en état "init" si la connexion précédente est morte */
	send_at("AT#MQDISC=1");
	wait_for("OK", 3000);

	send_at("AT#MQCONN=1,\"" MQTT_CLIENT_ID "\",\"" MQTT_USER "\",\"" MQTT_PASS "\"");
	if (wait_for("OK", 30000) != 0) {
		printk("[MQTT] Echec MQCONN\n");
		return -1;
	}

	printk("[MQTT] Connecte\n");
	return 0;
}

/* ── Publication MQTT des résultats WiFi ────────────────────────────── */
static void mqtt_publish_wifi(void)
{
	char cmd[MQTT_BUF_SIZE + 80];
	const char *payload = mqtt_payload[0] ? mqtt_payload : "no_results";

	/* AT#MQPUBS=<inst>,<topic>,<retain>,<qos>,<message> */
	snprintf(cmd, sizeof(cmd),
		 "AT#MQPUBS=1,\"" MQTT_TOPIC_WIFI "\",1,0,\"%s\"", payload); /* retain=1 */
	send_at(cmd);

	if (wait_for("OK", 10000) != 0) {
		printk("[MQTT] Echec publication\n");
	} else {
		printk("[MQTT] Publie sur " MQTT_TOPIC_WIFI "\n");
	}
}

/* ── Publication MQTT des données GNSS ─────────────────────────────── */
static void mqtt_publish_gnss(void)
{
	char cmd[128];

	snprintf(cmd, sizeof(cmd),
		 "AT#MQPUBS=1,\"" MQTT_TOPIC_GNSS "\",1,0,\"%s\"", gnss_payload);
	send_at(cmd);

	if (wait_for("OK", 10000) != 0) {
		printk("[MQTT] Echec publication GNSS\n");
	} else {
		printk("[MQTT] Publie sur " MQTT_TOPIC_GNSS " : %s\n", gnss_payload);
	}
}

/* ── Déconnexion MQTT ───────────────────────────────────────────────── */
static void mqtt_disconnect(void)
{
	send_at("AT#MQDISC=1");
	wait_for("OK", 5000);
	printk("[MQTT] Deconnecte\n");
}

/* ── Main ─────────────────────────────────────────────────────────── */
int main(void)
{
	/* --- Init GPIO : DTR démarre HIGH (module actif) --- */
	if (!device_is_ready(gpio1_dev)) {
		printk("GPIO1 non prêt\n");
		return -1;
	}
	gpio_pin_configure(gpio1_dev, DTR_PIN, GPIO_OUTPUT_LOW);
	dtr_set(1);

	if (!device_is_ready(telit_uart)) {
		printk("UART non prêt\n");
		return -1;
	}
	uart_irq_callback_user_data_set(telit_uart, telit_rx_cb, NULL);
	uart_irq_rx_enable(telit_uart);

	k_msleep(5000); /* attente démarrage du module */

	/* --- Vérification basique --- */
	send_at("AT");
	wait_for("OK", 3000);

	/* --- Activation des messages d'erreur verbeux --- */
	send_at("AT+CMEE=2");
	wait_for("OK", 3000);

	/* --- Mappage GPIO1 du module sur le signal DTR (ALT8 = DTR sur GPIO_01) --- */
	send_at("AT#GPIO=1,0,9");
	wait_for("OK", 3000);

	/* --- Configuration de l'APN --- */
	send_at("AT+CGDCONT=1,\"IP\",\"iot.1nce.net\"");
	wait_for("OK", 5000);

	/* --- Activation du contexte PDP (ignoré si déjà actif) --- */
	send_at("AT#SGACT?");
	if (wait_for("#SGACT: 1,1", 3000) != 0) {
		send_at("AT#SGACT=1,1");
		if (wait_for("#SGACT:", 30000) != 0) {
			printk("Attention : timeout SGACT\n");
		}
		wait_for("OK", 5000);
	} else {
		printk("Contexte PDP déjà actif\n");
		wait_for("OK", 1000);
	}

	/* --- Synchronisation automatique de l'heure via le réseau (NITZ) --- */
	send_at("AT+CTZU=1");
	wait_for("OK", 3000);
	send_at("AT#NITZ=7");
	wait_for("OK", 3000);

	/* --- Activation eDRX : CAT-M (act-type=4), cycle ~10.24s (0011) --- */
	send_at("AT#CEDRXS=2,4,\"0011\"");
	wait_for("OK", 3000);

	/* --- Configuration WiFi initiale --- */
	wifi_init();

	/* --- Initialisation MQTT+SSL en CFUN=1 (radio pleine) avant eDRX ---
	 * MQCFG échoue systématiquement en CFUN=5 ("connection failed") :
	 * on configure le broker ici pendant que CFUN=1 est encore actif. */
	mqtt_init();

	/* --- Activation eDRX piloté par DTR ---
	 * CFUN=5 : module entre en veille dès que DTR passe au niveau "sleep".
	 * Après CFUN=5, un front DTR sleep→wake est nécessaire pour démarrer
	 * correctement le premier cycle (le module attend l'edge, pas un niveau).
	 */
	send_at("AT+CFUN=5");
	wait_for("OK", 5000);
	dtr_set(0);        /* front descendant : DTR -> "sleep" */
	k_msleep(200);
	dtr_set(1);        /* front montant  : DTR -> "wake"  → module actif */
	k_msleep(3000);

	/* ── Boucle principale : scan -> MQTT -> veille -> réveil -> répéter ── */
	while (1) {
		wifi_scan_cycle();
		gnss_tracking_cycle();

		if (mqtt_connect() == 0) {
			mqtt_publish_wifi();
			mqtt_disconnect();
		}

		print_next_scan_time();

		printk("\n--- Passage en veille (DTR LOW) - réveil dans %d s (%d min) ---\n",
		       SLEEP_MS / 1000, SLEEP_MS / 60000);
		dtr_set(0);
		k_msleep(SLEEP_MS);

		printk("--- Réveil (DTR HIGH) ---\n");
		dtr_set(1);
		k_msleep(3000); /* laisse le temps au module de se réveiller */
		wifi_init();    /* re-initialise le WiFi après la veille DTR */
	}

	return 0;
}
