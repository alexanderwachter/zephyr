/*
 * Copyright (c) 2021 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/spi.h>
#include <drivers/sensor.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(INA229, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT ti_ina229

#define INA229_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)
#define INA229_SPI_READ       (1 << 0)
#define INA229_SPI_WRITE      (0 << 0)
#define INA229_SPI_ADDR(addr) (addr << 2)

#define INA229_REG_CONFIG_1        0x00
#define INA229_REG_ADCCONFIG_2     0x01
#define INA229_REG_CURRLSBCALC_3   0x02
#define INA229_REG_TEMPCOCONFIG_4  0x03
#define INA229_REG_VSHUNT          0x04
#define INA229_REG_VBUS            0x05
#define INA229_REG_DIETEMP         0x06
#define INA229_REG_CURRENT         0x07
#define INA229_REG_POWER           0x08
#define INA229_REG_ENERGY          0x09
#define INA229_REG_CHARGE          0x0a
#define INA229_REG_DIAG_ALRT       0x0b
#define INA229_REG_SOVL            0x0c
#define INA229_REG_SUVL            0x0d
#define INA229_REG_BOVL            0x0e
#define INA229_REG_BUVL            0x0f
#define INA229_REG_TEMP_LIMIT      0x10
#define INA229_REG_PWR_LIMIT       0x11
#define INA229_REG_MANUFACTURER_ID 0x3e
#define INA229_REG_DEVICE_ID       0x3f

#define INA229_MODE_SHUTDOWN              0x0
#define INA229_MODE_TRIG_VBUS             0x1
#define INA229_MODE_TRIG_VSHUNT           0x2
#define INA229_MODE_TRIG_VBUS_VSHUNT      0x3
#define INA229_MODE_TRIG_TEMP             0x4
#define INA229_MODE_TRIG_VBUS_VSHUNT_TEMP 0x7
#define INA229_MODE_CONT_VBUS_SHUNT       0xb
#define INA229_MODE_CONT_VBUS_VSHUNT_TEMP 0xf

union ina229_config_1 {
	struct {
		uint16_t reserved   : 4;
		uint16_t adc_range  : 1; /* 1: ADC range  40.96 mV, else 163.84 mV */
		uint16_t tempcomp   : 1; /* enable temperature compensation */
		uint16_t conv_delay : 8; /* inital conversion delay */
		uint16_t reset_acc  : 1; /* Reset charge and energy */
		uint16_t reset      : 1; /* Reset */
	}__packed;

	uint16_t reg;
};

union ina229_adcconfig_2 {
	struct {
		uint16_t avg    : 3; /* number of samples for averaging */
		uint16_t vtct   : 3; /* Temperature conversion time */
		uint16_t vshct  : 3; /* Vshunt (current) conversion time */
		uint16_t vbusct : 3; /* Vbus conversion time */
		uint16_t mode   : 4;
	}__packed;

	uint16_t reg;
};
union ina229_dia_alrt {
	struct {
		uint16_t memstat : 1; /* CRC error in trim memory */
		uint16_t cnvrf   : 1; /* Conversion completed */
		uint16_t pol     : 1; /* Power limit exceeds the threshold */
		uint16_t busul   : 1; /* Vbus below threshold */
		uint16_t busol   : 1; /* Vbus exceeds threshold */
		uint16_t shuntul : 1; /* Vshunt (current) below threshold */
		uint16_t shuntol : 1; /* Vshunt (current) exceeds threshold */
		uint16_t tmpol   : 1; /* Temperature exceeds threshold  */
		uint16_t res     : 1;
		uint16_t movf    : 1; /* Mathematic overflow (Power/Curren) */
		uint16_t chrof   : 1; /* Charge counter overflow */
		uint16_t enrgof  : 1; /* Energy counter overflow */
		uint16_t apol    : 1; /* Alert pin polarity */
		uint16_t slwalrt : 1; /* Alerts on averaged values */
		uint16_t cnvr    : 1; /* Assert alert pin on conversion ready */
		uint16_t alrlen  : 1; /* Latch alerts until register is read */
	}__packed;

	uint16_t reg;
};

const union ina229_config_1 reset_frame = {
	.reset = 1
};

static const uint8_t manufacturer_id[] = {'T', 'I'};

struct ina229_config {
	const struct device* spi_dev;
	struct spi_config spi_cfg;
	uint16_t vbus_ct;
	uint16_t vshunt_ct;
#ifdef CONFIG_INA229_TEMP_COMP
	uint16_t temp_ct;
#endif
	uint16_t averaging;
	uint32_t resistor;
	uint32_t max_current;
#ifdef CONFIG_INA229_TEMP_COMP
	uint16_t temp_comp;
#endif
	uint8_t  adc_range_narrow : 1;
};

struct ina229_data {
#ifdef CONFIG_INA229_CONT_MODE
	uint64_t charge;
	uint64_t energy;
#endif
	int32_t power;
	int32_t voltage;
	int32_t current;
#if defined(CONFIG_INA229_TEMP_COMP) || !defined(CONFIG_INA229_CONT_MODE)
	int16_t temp;
#endif
};

static const uint16_t avg_table[] = {1, 4, 16, 64, 128, 256, 512};
static const uint16_t ct_table[] = {50, 84, 150, 280, 540, 1052, 2074, 4120};



static int table_to_reg(uint16_t val, const uint16_t *table,
			size_t table_size)
{
	for (size_t index = 0; index < table_size; index++) {
		if (table[index] == val) {
			return table[index];
		}
	}

	return -1;
}

#define DEV_CFG(dev) ((const struct ina229_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct ina229_data *const)(dev)->data)

static int ina229_read(const struct ina229_config *cfg, uint8_t addr,
		       uint8_t *buf, uint8_t len)
{
	uint8_t addr_buf = INA229_SPI_READ | INA229_SPI_ADDR(addr);

	struct spi_buf tx_buf[] = {
		{ .buf = &addr_buf, .len = sizeof(addr_buf) },
		{ .buf = NULL, .len = len }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr_buf) },
		{ .buf = buf, .len = len }
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)
	};

	return spi_transceive(cfg->spi_dev, &cfg->spi_cfg, &tx, &rx);
}

static int ina229_write(const struct ina229_config *cfg, uint8_t addr,
			uint16_t data)
{
	uint8_t cmd_buf = INA229_SPI_WRITE | INA229_SPI_ADDR(addr);

	struct spi_buf tx_buf[] = {
		{ .buf = &cmd_buf, .len = sizeof(cmd_buf) },
		{ .buf = &data, .len = sizeof(data) }
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)
	};

	return spi_write(cfg->spi_dev, &cfg->spi_cfg, &tx);
}

static int ina229_read_alert(const struct ina229_config *cfg,
			     union ina229_dia_alrt *alerts)
{
	uint8_t buf[2];
	int ret;

	ret = ina229_read(cfg, INA229_REG_DIAG_ALRT, buf, sizeof(buf));
	if (ret != 0) {
		return ret;
	}

	alerts->reg = (buf[0] << 8U) | buf[1];

	return 0;
}

static int ina229_set_adccfg(const struct ina229_config *cfg, uint8_t mode)
{
	union ina229_adcconfig_2 adc_cfg;
	int ret;

	ret = table_to_reg(cfg->averaging, avg_table, sizeof(avg_table));
	adc_cfg.avg = (uint16_t)ret;
	if (ret < 0) {
		return -EINVAL;
	}
#ifdef CONFIG_INA229_TEMP_COMP
	ret = table_to_reg(cfg->temp_ct, ct_table, sizeof(ct_table));
	adc_cfg.vtct = (uint16_t)ret;
	if (ret < 0) {
		return -EINVAL;
	}
#else
	adc_cfg.vtct = 0;
#endif
	ret = table_to_reg(cfg->vshunt_ct, ct_table, sizeof(ct_table));
	adc_cfg.vshct = (uint16_t)ret;
	if (ret < 0) {
		return -EINVAL;
	}

	ret = table_to_reg(cfg->vbus_ct, ct_table, sizeof(ct_table));
	adc_cfg.vbusct = (uint16_t)ret;
	if (ret < 0) {
		return -EINVAL;
	}

	adc_cfg.mode = mode;

	return ina229_write(cfg, INA229_REG_ADCCONFIG_2, adc_cfg.reg);
}

static int ina229_read_be24(const struct ina229_config *cfg, uint8_t addr,
			    int32_t *result)
{
	int32_t data;
	int ret;
	uint8_t buf[3];

	ret = ina229_read(cfg, addr, buf, sizeof(buf));
	if (ret != 0) {
		return ret;
	}

	/* Values are in two's complement. Adjust left and device to
	 * preserve sign
	 */
	data = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8);
	*result = data / (1 << (3 + 8));

	return 0;
}

static int ina229_read_be40(const struct ina229_config *cfg, uint8_t addr,
			    int64_t *result)
{
	int64_t data;
	int ret;
	uint8_t buf[5];

	ret = ina229_read(cfg, addr, buf, sizeof(buf));
	if (ret != 0) {
		return ret;
	}

	/* Values are in two's complement. Adjust left and device to
	 * preserve sign
	 */
	data = ((uint64_t)buf[0] << 56ULL) | ((uint64_t)buf[1] << 48ULL) |
		((uint64_t)buf[2] << 40ULL) | ((uint64_t)buf[3] << 32ULL) |
		((uint64_t)buf[4] << 24ULL);
	*result = data / (1 << (64 - 40));

	return 0;
}

static int ina229_read_vbus(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	struct ina229_data *data = DEV_DATA(dev);
	int ret;

	ret = ina229_read_be24(cfg, INA229_REG_VBUS, &data->voltage);
	if (ret != 0) {
		LOG_ERR("Failed to read VBUS [%d]", ret);
	}

	return ret;
}

static int ina229_read_current(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	struct ina229_data *data = DEV_DATA(dev);
	int ret;

	ret = ina229_read_be24(cfg, INA229_REG_CURRENT, &data->current);
	if (ret != 0) {
		LOG_ERR("Failed to read current [%d]", ret);
	}

	return ret;
}

#ifdef CONFIG_INA229_TEMP_COMP
static int ina229_read_temperature(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	struct ina229_data *data = DEV_DATA(dev);
	uint8_t buf[2];
	int ret;

	ret = ina229_read(cfg, INA229_REG_DIETEMP, buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("Failed to read die temp [%d]", ret);
		return ret;
	}

	data->temp = (buf[1] << 8) | buf[0];

	return ret;
}
#endif

static int ina229_read_power(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	struct ina229_data *data = DEV_DATA(dev);
	int ret;

	ret = ina229_read_be24(cfg, INA229_REG_POWER, &data->power);
	if (ret != 0) {
		LOG_ERR("Failed to read power [%d]", ret);
	}

	return ret;
}

#ifdef CONFIG_INA229_CONT_MODE
static int ina229_read_charge(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	struct ina229_data *data = DEV_DATA(dev);
	int ret;

	ret = ina229_read_be40(cfg, INA229_REG_CHARGE, &data->charge);
	if (ret != 0) {
		LOG_ERR("Failed to read power [%d]", ret);
	}

	return ret;
}

static int ina229_read_energy(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	struct ina229_data *data = DEV_DATA(dev);
	int ret;

	ret = ina229_read_be40(cfg, INA229_REG_ENERGY, &data->energy);
	if (ret != 0) {
		LOG_ERR("Failed to read power [%d]", ret);
	}

	return ret;
}
#endif /* CONFIG_INA229_CONT_MODE */

#ifndef CONFIG_INA229_CONT_MODE
static int ina229_start_conversion(const struct device *dev,
				   enum sensor_channel chan)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	uint8_t mode;

	switch(chan) {
	case SENSOR_CHAN_VOLTAGE:
		mode = INA229_MODE_TRIG_VBUS;
		break;
	case SENSOR_CHAN_CURRENT:
		mode = INA229_MODE_TRIG_VSHUNT;
		break;
	case SENSOR_CHAN_POWER:
		mode = INA229_MODE_TRIG_VBUS_VSHUNT;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		mode = INA229_MODE_TRIG_TEMP;
	case SENSOR_CHAN_ALL:
		mode = INA229_MODE_TRIG_VBUS_VSHUNT_TEMP;
	default:
		return -ENOTSUP;
	}

	return ina229_set_adccfg(cfg, mode);
}
#endif

static uint32_t ina229_sample_time(const struct ina229_config *cfg,
				   enum sensor_channel chan)
{
	uint32_t time = 0;

	if (chan == SENSOR_CHAN_VOLTAGE || chan == SENSOR_CHAN_POWER ||
	    chan == SENSOR_CHAN_ALL) {
		time += cfg->vbus_ct;
	}

	if (chan == SENSOR_CHAN_CURRENT || chan == SENSOR_CHAN_POWER ||
	    chan == SENSOR_CHAN_ALL) {
		time += cfg->vshunt_ct ;
	}

	if (chan == SENSOR_CHAN_DIE_TEMP || chan == SENSOR_CHAN_ALL) {
		time += cfg->temp_ct;
	}

	time *= cfg->averaging;

	return time;
}

static int ina229_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	int ret;
	uint32_t conversion_time = ina229_sample_time(DEV_CFG(dev), chan);

#ifndef CONFIG_INA229_CONT_MODE
	ret = ina229_start_conversion(dev, chan);
	if (ret != 0) {
		LOG_ERR("Could not start conversion [%d]");
		return ret;
	}

	k_usleep(conversion_time)
#endif
	for (int tries = 0;; tries++) {
		union ina229_dia_alrt alrt;
		ret = ina229_read_alert(DEV_CFG(dev), &alrt);
		if (ret != 0) {
			LOG_ERR("Failed to read conversion ready");
			return ret;
		}
		if (alrt.cnvrf) {
			break;
		}

		if (tries > 10 ) {
			LOG_ERR("Conversion not ready");
			return -EAGAIN;
		}

		k_usleep(conversion_time / 10);
	}

	switch(chan) {
	case SENSOR_CHAN_VOLTAGE:
		return ina229_read_vbus(dev);

	case SENSOR_CHAN_CURRENT:
		return ina229_read_current(dev);

	case SENSOR_CHAN_POWER:
		return ina229_read_power(dev);

#if defined(CONFIG_INA229_TEMP_COMP) || !defined(CONFIG_INA229_CONT_MODE)
	case SENSOR_CHAN_DIE_TEMP:
		return ina229_read_temperature(dev);
#endif
#ifdef CONFIG_INA229_CONT_MODE
	case SENSOR_CHAN_CHARGE:
		return ina229_read_charge(dev);

	case SENSOR_CHAN_ENERGY:
		return ina229_read_energy(dev);
#endif
	case SENSOR_CHAN_ALL:
		ret = ina229_read_vbus(dev);
		ret |= ina229_read_current(dev);
#if defined(CONFIG_INA229_TEMP_COMP) || !defined(CONFIG_INA229_CONT_MODE)
		ret |= ina229_read_temperature(dev);
#endif
		ret |= ina229_read_power(dev);
#ifdef CONFIG_INA229_CONT_MODE
		ret |= ina229_read_charge(dev);
		ret |= ina229_read_energy(dev);
#endif
		if (ret != 0) {
			return -EIO;
		}

		return 0;

	default:
		return -ENOTSUP;
	}
}

static inline void ina229_convert_voltage(int32_t voltage_raw,
					  struct sensor_value *val)
{
	/* Conversion factor: 195.3125 μV/LSB */
	int32_t voltage = ((int64_t)voltage_raw * 1953125LL) / 10000LL;
	val->val1 = voltage / 1000000L;
	val->val2 = voltage % 1000000L;
}

static inline void ina229_convert_current(int32_t current_raw,
					  const struct ina229_config *cfg,
					  struct sensor_value *val)
{
	/* The current LSB is the max current / 2^19
	 * the max current is in mA already hence multiply with 1000 to get uA
	 */
	int32_t current_uA = ((int64_t)current_raw * 1000LL *
			     (int64_t)cfg->max_current) >> 19;
	val->val1 = current_uA / 1000000L;
	val->val2 = current_uA % 1000000L;
}

static inline void ina229_convert_power(int32_t power_raw,
					const struct ina229_config *cfg,
					struct sensor_value *val)
{
	/* Power [W] = 3.2 x CURRENT_LSB x POWER 
	 * The current LSB is the max current / 2^19
	 * the max current is in mA already hence multiply with 3200 to get uW
	 */
	int32_t power_uW = ((int64_t)power_raw * 3200LL *
			     (int64_t)cfg->max_current) >> 19;
	val->val1 = power_uW / 1000000L;
	val->val2 = power_uW % 1000000L;
}

#ifdef CONFIG_INA229_CONT_MODE
static inline void ina229_convert_energy(int64_t energy_raw,
					 const struct ina229_config *cfg,
					 struct sensor_value *val)
{
	/* Energy [J] = 16 x 3.2 x CURRENT_LSB x ENERGY
	 * The current LSB is the max current / 2^19
	 * the max current is in mA already hence multiply with 51200 to get uJ
	 */
	int64_t energy_uJ = (energy_raw * 51200LL *
			   (int64_t)cfg->max_current) >> 19;
	val->val1 = energy_uJ / 1000000LL;
	val->val2 = energy_uJ % 1000000LL;
}

static inline void ina229_convert_charge(int64_t charge_raw,
					 const struct ina229_config *cfg,
					 struct sensor_value *val)
{
	/* Charge [C] = CURRENT_LSB x CHARGE
	 * The current LSB is the max current / 2^19
	 * the max current is in mA already hence multiply with 1000 to get uC
	 */
	int64_t energy_uJ = (charge_raw * 1000LL *
			   (int64_t)cfg->max_current) >> 19;
	val->val1 = energy_uJ / 1000000LL;
	val->val2 = energy_uJ % 1000000LL;
}
#endif
#if defined(CONFIG_INA229_TEMP_COMP) || !defined(CONFIG_INA229_CONT_MODE)
static inline void ina229_convert_temperature(uint32_t temperature_raw,
					      struct sensor_value *val)
{
	/* Conversion factor: 7.8125 m°C/LSB */
	int32_t temp = (temperature_raw * 78125L) / 10L;
	val->val1 = temp / 1000000L;
	val->val2 = temp % 1000000L;
}
#endif

static int ina229_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ina229_data *data = DEV_DATA(dev);

	switch(chan) {
	case SENSOR_CHAN_VOLTAGE:
		ina229_convert_voltage(data->voltage, val);
		break;

	case SENSOR_CHAN_CURRENT:
		ina229_convert_current(data->current, DEV_CFG(dev), val);
		break;

	case SENSOR_CHAN_POWER:
		ina229_convert_current(data->power, DEV_CFG(dev), val);
		break;

#if defined(CONFIG_INA229_TEMP_COMP) || !defined(CONFIG_INA229_CONT_MODE)
	case SENSOR_CHAN_DIE_TEMP:
		ina229_convert_temperature(data->temp, val);
		break;
#endif
#ifdef CONFIG_INA229_CONT_MODE
	case SENSOR_CHAN_CHARGE:
		ina229_convert_charge(data->charge, DEV_CFG(dev), val);
		break;

	case SENSOR_CHAN_ENERGY:
		ina229_convert_energy(data->energy, DEV_CFG(dev), val);
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ina229_init(const struct device *dev)
{
	const struct ina229_config *cfg = DEV_CFG(dev);
	const union ina229_config_1 cfg_1 = {
		.reserved = 0,
		.adc_range = cfg->adc_range_narrow,
#if CONFIG_INA229_TEMP_COMP
		.tempcomp = cfg->temp_comp > 0 ? 1 : 0,
#else
		.tempcomp = 0,
#endif
		.conv_delay = 0,
		.reset_acc = 1,
		.reset = 0,
	};
	const union ina229_dia_alrt alrt = {
		.alrlen = 1
	};
	int ret;
	uint16_t id;
	uint32_t curr_lsb_calc;
	uint8_t adc_mode;

#if defined(CONFIG_INA229_CONT_MODE) && defined(CONFIG_INA229_TEMP_COMP)
	adc_mode = INA229_MODE_CONT_VBUS_VSHUNT_TEMP;
#elif CONFIG_INA229_CONT_MODE
	adc_mode = INA229_MODE_CONT_VBUS_VSHUNT;
#else
	adc_mode = INA229_MODE_SHUTDOWN;
#endif

	ret = ina229_read(cfg, INA229_REG_MANUFACTURER_ID, (uint8_t*)&id,
			  sizeof(id));
	if (ret != 0 || id != *manufacturer_id) {
		LOG_ERR("Failed to read man. id [%d]. [%d]", id, ret);
		return -EIO;
	}

	ret = ina229_write(cfg, INA229_REG_CONFIG_1, reset_frame.reg);
	if (ret) {
		LOG_ERR("Failed to send reset [%d]", ret);
		return ret;
	}

#ifdef CONFIG_INA229_TEMP_COMP
	ret = ina229_write(cfg, INA229_REG_TEMPCOCONFIG_4, cfg->temp_comp);
	if (ret) {
		LOG_ERR("Failed to set temp coeff [%d]", ret);
		return ret;
	}
#endif
	ret = ina229_write(cfg, INA229_REG_CONFIG_1, cfg_1.reg);
	if (ret) {
		LOG_ERR("Failed to set cfg [%d]", ret);
		return ret;
	}

	/* CURRLSBCALC = 13107.2E6 * CURRENT_LSB * Rshunt 
	 * CURR_LSB = max expected current / 2^19 
	 * max current from device tree is in mA, resistor is in mOhm,
	 * hence the E6 from the constant is canceled out.
	 * Multiply the constant by 10 and divide by 10 again for precision
	 */
	curr_lsb_calc = (131072ULL * (uint64_t)cfg->max_current *
			(uint64_t)cfg->resistor / 10ULL) >> 19ULL;

	if (curr_lsb_calc > 0xffff) {
		LOG_ERR("Resistor and max current combination is invalid");
		return -EINVAL;
	}

	ret = ina229_write(cfg, INA229_REG_CURRLSBCALC_3,
			   (uint16_t)curr_lsb_calc);
	if (ret) {
		LOG_ERR("Failed to write CURRLSB_CALC [%d]", ret);
		return ret;
	}

	ret = ina229_write(cfg, INA229_REG_DIAG_ALRT, alrt.reg);
	if (ret) {
		LOG_ERR("Failed to write DIAG_ALRT  [%d]", ret);
		return ret;
	}

	ret = ina229_set_adccfg(cfg, adc_mode);
	if (ret) {
		LOG_ERR("Failed to set ADC cfg [%d]", ret);
		return ret;
	}

	return 0;
}

static const struct sensor_driver_api ina229_api_funcs = {
	.sample_fetch = ina229_sample_fetch,
	.channel_get = ina229_channel_get,
};

#define INA229_CFG_INST_DEFINE(inst)                                   \
static const struct ina229_config ina229_cfg_##inst = {                \
	.spi_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                   \
	.spi_cfg = SPI_CONFIG_DT_INST(inst, INA229_SPI_OPERATION, 0),  \
	.vbus_ct = DT_INST_PROP(inst, vbus_ct),                        \
	.vshunt_ct = DT_INST_PROP(inst, vshunt_ct),                    \
	.temp_ct = DT_INST_PROP(inst, temp_ct),                        \
	.averaging = DT_INST_PROP(inst, averaging),                    \
	.resistor = DT_INST_PROP(inst, res),                           \
	.max_current = DT_INST_PROP(inst, max_current),                \
	.temp_comp = DT_INST_PROP_OR(inst, temp_comp, 0),              \
	.adc_range_narrow = DT_INST_PROP_OR(inst, adc_range_narrow, 0) \
}

#define INA229_DATA_INST_DEFINE(inst) \
static struct ina229_data ina229_data_##inst;

#define INA229_INST_DEFINE(inst)                                          \
INA229_CFG_INST_DEFINE(inst);                                             \
INA229_DATA_INST_DEFINE(inst);                                            \
DEVICE_DT_INST_DEFINE(inst, &ina229_init, device_pm_control_nop,          \
		    &ina229_data_##inst, &ina229_cfg_##inst, POST_KERNEL, \
		    CONFIG_SENSOR_INIT_PRIORITY, &ina229_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(INA229_INST_DEFINE)
