struct msc313_mux_data {
	const char *name;
	const struct clk_parent_data *parents;
	u8 num_parents;
	int offset;
	int gate_shift;
	int mux_shift;
	int mux_width;
	int deglitch_shift;
};

#define MSC313_MUX_DATA(_name, _parents, _offset, _gate_shift, _mux_shift, _mux_width, _deglitch_shift) \
	{						\
		.name = _name,				\
		.parents = _parents,			\
		.num_parents = ARRAY_SIZE(_parents),	\
		.offset = _offset,			\
		.gate_shift = _gate_shift,		\
		.mux_shift = _mux_shift,		\
		.mux_width = _mux_width,		\
		.deglitch_shift = _deglitch_shift,	\
	}

struct msc313_muxes_data {
	unsigned int num_muxes;
	const struct msc313_mux_data *muxes;
};

#define MSC313_MUXES_DATA(_muxes) \
	{ \
		.num_muxes = ARRAY_SIZE(_muxes), \
		.muxes = _muxes, \
	}

int msc313_mux_register_muxes(struct device *dev, struct regmap *regmap, const struct msc313_muxes_data *muxes_data);
