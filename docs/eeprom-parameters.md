# Outdoor unit EEPROM parameters

Registers 2999-3510 of the outdoor unit (Modbus address 2). Descriptions marked *(firmware v1.10)* were derived from a firmware analysis of outdoor unit program version 0x0110 (register 2122).

- Each register is stored as one byte in an external I2C EEPROM, so temperatures use the form Value = T + 30.
- A Modbus write only changes RAM. The value is lost after a power cycle of the outdoor unit.
- Saving to the EEPROM requires working mode 5 on 3999 followed by 4002 = 3, and a valid CRC-16/Modbus over 2999-3508 in 3509 (low byte) and 3510 (high byte). OpenAmber does not do this.

| Register | Default | Explanation |
|---:|---:|---|
| 2999 | 255 | EEPROM data length |
| 3000 | 0 | Cooling frequency F0 (Range: 0-120) |
| 3001 | 30 | Cooling frequency F1 (Range: 0-120) |
| 3002 | 36 | Cooling frequency F2 (Range: 0-120) |
| 3003 | 42 | Cooling frequency F3 (Range: 0-120) |
| 3004 | 47 | Cooling frequency F4 (Range: 0-120) |
| 3005 | 52 | Cooling frequency F5 (Range: 0-120) |
| 3006 | 56 | Cooling frequency F6 (Range: 0-120) |
| 3007 | 61 | Cooling frequency F7 (Range: 0-120) |
| 3008 | 66 | Cooling frequency F8 (Range: 0-120) |
| 3009 | 71 | Cooling frequency F9 (Range: 0-120) |
| 3010 | 74 | Cooling frequency F10 (Range: 0-120) |
| 3011 | 0 | Heating frequency F0 (Range: 0-120) |
| 3012 | 36 | Heating frequency F1 (Range: 0-120) |
| 3013 | 43 | Heating frequency F2 (Range: 0-120) |
| 3014 | 49 | Heating frequency F3 (Range: 0-120) |
| 3015 | 55 | Heating frequency F4 (Range: 0-120) |
| 3016 | 61 | Heating frequency F5 (Range: 0-120) |
| 3017 | 67 | Heating frequency F6 (Range: 0-120) |
| 3018 | 72 | Heating frequency F7 (Range: 0-120) |
| 3019 | 79 | Heating frequency F8 (Range: 0-120) |
| 3020 | 85 | Heating frequency F9 (Range: 0-120) |
| 3021 | 90 | Heating frequency F10 (Range: 0-120) |
| 3022 | 55 | Internal parameters |
| 3023 | 55 | Internal parameters |
| 3024 | 0 | Internal parameters |
| 3025 | 0 | Internal parameters |
| 3026 | 0 | Internal parameters |
| 3027 | 0 | Internal parameters |
| 3028 | 3 | EEV control mode (Range: 0: By master Control; 1: Point-to-Point Control; 2: Reserved; 3: PID;) |
| 3029 | 165 | Internal parameters |
| 3030 | 155 | Internal parameters |
| 3031 | 135 | Internal parameters |
| 3032 | 140 | Internal parameters |
| 3033 | 135 | Internal parameters |
| 3034 | 120 | Internal parameters |
| 3035 | 100 | Internal parameters |
| 3036 | 95 | Internal parameters |
| 3037 | 65 | Internal parameters |
| 3038 | 65 | Internal parameters |
| 3039 | 165 | Internal parameters |
| 3099 | 210 | Internal parameters |
| 3100 | 210 | Internal parameters |
| 3101 | 202 | Internal parameters |
| 3102 | 202 | Internal parameters |
| 3103 | 195 | Internal parameters |
| 3104 | 195 | Internal parameters |
| 3105 | 182 | Internal parameters |
| 3106 | 182 | Internal parameters |
| 3107 | 172 | Internal parameters |
| 3108 | 172 | Internal parameters |
| 3109 | 130 | Internal parameters |
| 3110 | 130 | Internal parameters |
| 3111 | 120 | Internal parameters |
| 3112 | 120 | Internal parameters |
| 3113 | 25 | Internal parameters |
| 3114 | 25 | Internal parameters |
| 3115 | 25 | Internal parameters |
| 3116 | 25 | Internal parameters |
| 3117 | 25 | Internal parameters |
| 3118 | 25 | Internal parameters |
| 3119 | 140 | Internal parameters |
| 3120 | 140 | Internal parameters |
| 3121 | 135 | Internal parameters |
| 3122 | 135 | Internal parameters |
| 3123 | 120 | Internal parameters |
| 3124 | 120 | Internal parameters |
| 3125 | 100 | Internal parameters |
| 3126 | 100 | Internal parameters |
| 3127 | 100 | Internal parameters |
| 3128 | 100 | Internal parameters |
| 3129 | 162 | Internal parameters |
| 3130 | 162 | Internal parameters |
| 3131 | 155 | Internal parameters |
| 3132 | 155 | Internal parameters |
| 3133 | 147 | Internal parameters |
| 3134 | 147 | Internal parameters |
| 3135 | 130 | Internal parameters |
| 3136 | 130 | Internal parameters |
| 3137 | 105 | Internal parameters |
| 3138 | 105 | Internal parameters |
| 3139 | 192 | Internal parameters |
| 3140 | 192 | Internal parameters |
| 3141 | 180 | Internal parameters |
| 3142 | 180 | Internal parameters |
| 3143 | 155 | Internal parameters |
| 3144 | 155 | Internal parameters |
| 3145 | 135 | Internal parameters |
| 3146 | 135 | Internal parameters |
| 3147 | 120 | Internal parameters |
| 3148 | 120 | Internal parameters |
| 3149 | 197 | Internal parameters |
| 3150 | 197 | Internal parameters |
| 3151 | 190 | Internal parameters |
| 3152 | 190 | Internal parameters |
| 3153 | 177 | Internal parameters |
| 3154 | 177 | Internal parameters |
| 3155 | 172 | Internal parameters |
| 3156 | 172 | Internal parameters |
| 3157 | 170 | Internal parameters |
| 3158 | 170 | Internal parameters |
| 3159 | 207 | Internal parameters |
| 3160 | 207 | Internal parameters |
| 3199 | 182 | Internal parameters |
| 3200 | 182 | Internal parameters |
| 3201 | 170 | Internal parameters |
| 3202 | 170 | Internal parameters |
| 3203 | 145 | Internal parameters |
| 3204 | 145 | Internal parameters |
| 3205 | 125 | Internal parameters |
| 3206 | 125 | Internal parameters |
| 3207 | 110 | Internal parameters |
| 3208 | 110 | Internal parameters |
| 3209 | 187 | Internal parameters |
| 3210 | 187 | Internal parameters |
| 3211 | 180 | Internal parameters |
| 3212 | 180 | Internal parameters |
| 3213 | 167 | Internal parameters |
| 3214 | 167 | Internal parameters |
| 3215 | 162 | Internal parameters |
| 3216 | 162 | Internal parameters |
| 3217 | 160 | Internal parameters |
| 3218 | 160 | Internal parameters |
| 3219 | 197 | Internal parameters |
| 3220 | 197 | Internal parameters |
| 3221 | 192 | Internal parameters |
| 3222 | 192 | Internal parameters |
| 3223 | 187 | Internal parameters |
| 3224 | 187 | Internal parameters |
| 3225 | 177 | Internal parameters |
| 3226 | 177 | Internal parameters |
| 3227 | 167 | Internal parameters |
| 3228 | 167 | Internal parameters |
| 3229 | 1 | Internal parameters |
| 3230 | 135 | Normal EEV opening when heating (steps = value × 2). *(firmware v1.10)* |
| 3231 | 135 | Internal parameters |
| 3232 | 0 | Internal parameters |
| 3233 | 0 | EEV setting during defrosting (Range: 0-450). Target = value × 2 steps. *(firmware v1.10)* |
| 3234 | 4 | Baud Rate (Range: 0: 1200; 1: 2400; 2: 4800; 3: 9600; 4: 19200;) |
| 3235 | 1 | Parity Check (Range: 0: Null 1 stop bit; 1: Even 1 stop bit; 2: Odd 1 stop bit;) |
| 3236 | 2 | Working of bottom plate heater (Range: 1: According to ambient temperature; 2: ON when defrosting starts, and OFF two minutes after defrosting ends) |
| 3237 | 34 | Ambient temperature to start bottom plate heater (Range: Value=T+30) |
| 3238 | 3 | Stop hysteresis of bottom plate heater (Range: 3℃) |
| 3239 | 28 | Ambient temperature to start crankcase heater (Range: Value=T+30) |
| 3240 | 3 | Stop hysteresis of crankcase heater (Range: 3℃) |
| 3241 | 0 | Internal parameters |
| 3242 | 0 | Internal parameters |
| 3243 | 0 | Internal parameters |
| 3244 | 1 | High pressure pressure sensor enable (Range: 0: No Enable; 1: Enable;) |
| 3245 | 1 | Low pressure pressure sensor enable (Range: 0: No Enable; 1: Enable;) |
| 3246 | 31 | High pressure protection shutdown value (Range: 41) |
| 3247 | 4 | Low pressure protection shutdown value (Range: 5) |
| 3248 | 1 | Fan control mode (Range: 0: By master control; 1: Control by outdoor unit) |
| 3249 | 0 | Fan Type (Range: 0: DC Fan; 1: AC Fan;) |
| 3250 | 40 | Fan speed during defrost pressure relief and recovery, × 10 rpm. *(firmware v1.10)* |
| 3251 | 44 | Internal parameters |
| 3252 | 48 | Internal parameters |
| 3253 | 160 | Internal parameters |
| 3254 | 140 | Internal parameters |
| 3255 | 120 | Internal parameters |
| 3256 | 160 | Internal parameters |
| 3257 | 140 | Internal control parameter, value differs per model (140 = Amber 65, 150 = Amber 95, 15 = Amber 120, 145 = HP-S 95). OpenAmber uses it for model detection. |
| 3258 | 120 | Internal parameters |
| 3259 | 138 | Discharge temperature Td: compressor stop with fault (Value=T+30, default 108°C). *(firmware v1.10)* |
| 3260 | 135 | Discharge temperature Td: strong frequency reduction (Value=T+30, default 105°C). *(firmware v1.10)* |
| 3261 | 127 | Discharge temperature Td: frequency reduction (Value=T+30, default 97°C). *(firmware v1.10)* |
| 3262 | 125 | Discharge temperature Td: no further frequency increase (Value=T+30, default 95°C). *(firmware v1.10)* |
| 3263 | 92 | Internal parameters |
| 3264 | 89 | Internal parameters |
| 3265 | 87 | Internal parameters |
| 3266 | 82 | Internal parameters |
| 3267 | 30 | High pressure Pd: strong frequency reduction (bar, default 30). *(firmware v1.10)* |
| 3268 | 28 | High pressure Pd: frequency reduction (bar, default 28). *(firmware v1.10)* |
| 3269 | 26 | High pressure Pd: no further frequency increase (bar, default 26). *(firmware v1.10)* |
| 3270 | 0 | Internal parameters |
| 3271 | 12 | Internal parameters |
| 3272 | 32 | Internal parameters |
| 3273 | 36 | Internal parameters |
| 3274 | 1 | Defrost control mode (Range: 0: By master control, master starts a defrost with working mode 4 on 3999; 1: Control by outdoor unit; other values: never defrost) |
| 3275 | 0 | Defrost mode, only used when 3274 = 1 (Range: 0: Smart Defrost, threshold per ambient band and adaptive 45-120 min interval; 1: Fixed defrost, single threshold and 45 min interval; 2: Reserved, no defrost; 3: AI Smart Defrost) |
| 3276 | 27 | Enter defrost temperature 1, coil temperature Tp threshold when Ta ≥ 3°C; also the only threshold for Fixed and AI Smart (Range: Value=T+30) |
| 3277 | 47 | Exit defrost temperature (Range: Value=T+30) |
| 3278 | 8 | Maximum defrosting time (Range: 8) |
| 3279 | 36 | Defrost Standby frequency (Range: 30) |
| 3280 | 61 | Defrost running frequency (Range: 61) |
| 3281 | 1 | Frequency limitation control for ambient outdoor temperature (Range: 0: By master control; 1: Control by outdoor unit, caps the requested level using 3282-3298) |
| 3282 | 0 | Cooling: max compressor level when Ta < 0°C. *(firmware v1.10)* |
| 3283 | 2 | Cooling: max compressor level when 0 ≤ Ta < 12°C. *(firmware v1.10)* |
| 3284 | 3 | Cooling: max compressor level when 12 ≤ Ta < 18°C. *(firmware v1.10)* |
| 3285 | 6 | Cooling: max compressor level when 18 ≤ Ta < 24°C. *(firmware v1.10)* |
| 3286 | 10 | Cooling: max compressor level when 24 ≤ Ta < 30°C. *(firmware v1.10)* |
| 3287 | 10 | Cooling: max compressor level when 30 ≤ Ta < 41°C. *(firmware v1.10)* |
| 3288 | 7 | Cooling: max compressor level when 41 ≤ Ta < 45°C. *(firmware v1.10)* |
| 3289 | 4 | Cooling: max compressor level when 45 ≤ Ta < 51°C. *(firmware v1.10)* |
| 3290 | 1 | Cooling: max compressor level when 51 ≤ Ta < 65°C. *(firmware v1.10)* |
| 3291 | 0 | Cooling: max compressor level when Ta ≥ 65°C. *(firmware v1.10)* |
| 3292 | 0 | Heating: max compressor level below the low ambient limit (3498 − 3499 − 30, default −25°C). *(firmware v1.10)* |
| 3293 | 10 | Heating: max compressor level from the low ambient limit up to Ta 12°C. *(firmware v1.10)* |
| 3294 | 8 | Heating: max compressor level when 12 ≤ Ta < 20°C. *(firmware v1.10)* |
| 3295 | 5 | Heating: max compressor level when 20 ≤ Ta < 24°C. *(firmware v1.10)* |
| 3296 | 3 | Heating: max compressor level when 24 ≤ Ta < 28°C. *(firmware v1.10)* |
| 3297 | 2 | Heating: max compressor level when 28 ≤ Ta < 46°C. *(firmware v1.10)* |
| 3298 | 0 | Heating: max compressor level when Ta ≥ 46°C. *(firmware v1.10)* |
| 3299 | 3 | Internal parameters |
| 3300 | 7 | Compressor/model code, sent to the inverter and part of register 2114 ((3495 << 8) + 3300). |
| 3301 | 52 | Internal parameters |
| 3302 | 53 | Internal parameters |
| 3303 | 56 | Internal parameters |
| 3304 | 62 | Internal parameters |
| 3305 | 68 | Internal parameters |
| 3306 | 72 | Internal parameters |
| 3307 | 1 | Low noise mode switch (Range: 0: No Enable; 1: Enable;) |
| 3308 | 62 | Low noise mode FAN speed value (Range: Value=R/10) |
| 3309 | 1 | Number of DC fan (Range: 0: Double fan; 1: Single fan;) |
| 3310 | 114 | EEPROM version, reported in register 2123. |
| 3311 | 10 | AI Smart defrost: ambient limit, active below value − 2°C and off above value + 4°C. *(firmware v1.10)* |
| 3312 | 0 | Internal parameters |
| 3313 | 0 | Internal parameters |
| 3314 | 0 | Internal parameters |
| 3315 | 40 | AI Smart defrost: deviation threshold (× 10) against the learned per fan step reference. *(firmware v1.10, hypothesis)* |
| 3316 | 2 | Setting of the model(main pcb) address (Range: Only N30(PCB) is set via this address) |
| 3317 | 0 | Internal parameters |
| 3318 | 0 | Internal parameters |
| 3319 | 61 | Internal parameters |
| 3320 | 52 | Internal parameters |
| 3321 | 52 | Internal parameters |
| 3322 | 52 | Internal parameters |
| 3323 | 62 | Internal parameters |
| 3324 | 62 | Internal parameters |
| 3325 | 62 | Internal parameters |
| 3326 | 72 | Internal parameters |
| 3327 | 72 | Internal parameters |
| 3328 | 72 | Internal parameters |
| 3329 | 140 | Internal parameters |
| 3330 | 156 | Internal parameters |
| 3331 | 170 | Internal parameters |
| 3332 | 188 | Internal parameters |
| 3333 | 2 | Internal parameters |
| 3334 | 5 | Internal parameters |
| 3335 | 6 | Internal parameters |
| 3336 | 27 | Enter defrost temperature 2, coil temperature Tp threshold when −3 ≤ Ta < 3°C (Range: Value=T+30) |
| 3337 | 25 | Enter defrost temperature 3, coil temperature Tp threshold when −10 ≤ Ta < −3°C (Range: Value=T+30) |
| 3338 | 18 | Enter defrost temperature 4, coil temperature Tp threshold when Ta < −10°C (Range: Value=T+30) |
| 3339 | 10 | Minimum defrost time in seconds before the exit temperature is checked (only when the outdoor unit decides). *(firmware v1.10)* |
| 3340 | 55 | Second exit defrost temperature, acts as upper limit (Value=T+30). Only when the outdoor unit decides. *(firmware v1.10)* |
| 3341 | 1 | Smart defrost: fixed 45 minute interval when −3 ≤ Ta < 3°C (0: off; 1: on). *(firmware v1.10)* |
| 3342 | 120 | Internal parameters |
| 3343 | 100 | Internal parameters |
| 3344 | 147 | Internal parameters |
| 3345 | 140 | Internal parameters |
| 3346 | 45 | Internal parameters |
| 3347 | 120 | Internal parameters |
| 3348 | 150 | Internal parameters |
| 3349 | 0 | Internal parameters |
| 3350 | 0 | Internal parameters |
| 3351 | 0 | Internal parameters |
| 3352 | 0 | Internal parameters |
| 3353 | 0 | Internal parameters |
| 3354 | 0 | Internal parameters |
| 3355 | 0 | Internal parameters |
| 3356 | 0 | Internal parameters |
| 3357 | 1 | Internal parameters |
| 3358 | 50 | Internal parameters |
| 3359 | 100 | Internal parameters |
| 3360 | 135 | Internal parameters |
| 3361 | 175 | Internal parameters |
| 3362 | 200 | Internal parameters |
| 3363 | 170 | Internal parameters |
| 3364 | 130 | Internal parameters |
| 3365 | 130 | Internal parameters |
| 3366 | 180 | Internal parameters |
| 3367 | 190 | Internal parameters |
| 3368 | 180 | Internal parameters |
| 3369 | 170 | Internal parameters |
| 3370 | 190 | Internal parameters |
| 3371 | 130 | Internal parameters |
| 3372 | 170 | Internal parameters |
| 3373 | 190 | Internal parameters |
| 3374 | 195 | Internal parameters |
| 3375 | 180 | Internal parameters |
| 3376 | 180 | Internal parameters |
| 3377 | 160 | Internal parameters |
| 3378 | 180 | Internal parameters |
| 3379 | 190 | Internal parameters |
| 3380 | 195 | Internal parameters |
| 3381 | 10 | Internal parameters |
| 3382 | 10 | Internal parameters |
| 3383 | 10 | Internal parameters |
| 3384 | 10 | Internal parameters |
| 3385 | 10 | Internal parameters |
| 3386 | 10 | Internal parameters |
| 3387 | 10 | Internal parameters |
| 3388 | 10 | Internal parameters |
| 3389 | 10 | Internal parameters |
| 3390 | 40 | Internal parameters |
| 3391 | 18 | Internal parameters |
| 3392 | 27 | Internal parameters |
| 3393 | 10 | Internal parameters |
| 3394 | 18 | Internal parameters |
| 3395 | 18 | Internal parameters |
| 3396 | 5 | Internal parameters |
| 3397 | 45 | Internal parameters |
| 3398 | 0 | Internal parameters |
| 3399 | 0 | Internal parameters |
| 3400 | 0 | Internal parameters |
| 3401 | 0 | Internal parameters |
| 3402 | 0 | Internal parameters |
| 3403 | 0 | Internal parameters |
| 3404 | 0 | Internal parameters |
| 3405 | 0 | Internal parameters |
| 3406 | 0 | Internal parameters |
| 3407 | 0 | Internal parameters |
| 3408 | 0 | Internal parameters |
| 3409 | 0 | Internal parameters |
| 3410 | 0 | Internal parameters |
| 3411 | 0 | Internal parameters |
| 3412 | 0 | Internal parameters |
| 3413 | 0 | Internal parameters |
| 3414 | 0 | Internal parameters |
| 3415 | 0 | Internal parameters |
| 3416 | 0 | Internal parameters |
| 3417 | 0 | Internal parameters |
| 3418 | 0 | Internal parameters |
| 3419 | 0 | Internal parameters |
| 3420 | 0 | Internal parameters |
| 3421 | 0 | Internal parameters |
| 3422 | 0 | Internal parameters |
| 3423 | 0 | Internal parameters |
| 3424 | 0 | Internal parameters |
| 3425 | 0 | Internal parameters |
| 3426 | 0 | Internal parameters |
| 3427 | 0 | Internal parameters |
| 3428 | 0 | Internal parameters |
| 3429 | 0 | Internal parameters |
| 3430 | 0 | Internal parameters |
| 3431 | 0 | Internal parameters |
| 3432 | 0 | Internal parameters |
| 3433 | 0 | Internal parameters |
| 3434 | 0 | Internal parameters |
| 3435 | 0 | Internal parameters |
| 3436 | 0 | Internal parameters |
| 3437 | 0 | Internal parameters |
| 3438 | 0 | Internal parameters |
| 3439 | 0 | Internal parameters |
| 3440 | 0 | Internal parameters |
| 3441 | 0 | Internal parameters |
| 3442 | 0 | Internal parameters |
| 3443 | 0 | Internal parameters |
| 3444 | 0 | Internal parameters |
| 3445 | 0 | Internal parameters |
| 3446 | 0 | Internal parameters |
| 3447 | 0 | Internal parameters |
| 3448 | 0 | Internal parameters |
| 3449 | 0 | Internal parameters |
| 3450 | 0 | Internal parameters |
| 3451 | 0 | Internal parameters |
| 3452 | 0 | Internal parameters |
| 3453 | 0 | Internal parameters |
| 3454 | 0 | Internal parameters |
| 3455 | 0 | Min. flow rate (Range: Different models with different values) |
| 3456 | 0 | Internal parameters |
| 3457 | 0 | L1, L2, L3 Relay control by master (Range: 0: No Enable; 1: Enable;) |
| 3458 | 0 | Water flow sensor type (Range: 0: No Enable; 1: sika; 2: huba) |
| 3459 | 0 | Internal parameters |
| 3460 | 15 | Internal parameters |
| 3461 | 44 | Internal parameters |
| 3462 | 30 | Internal parameters |
| 3463 | 12 | Internal parameters |
| 3464 | 45 | Internal parameters |
| 3465 | 42 | Internal parameters |
| 3466 | 97 | Internal parameters |
| 3467 | 45 | Internal parameters |
| 3468 | 80 | Internal parameters |
| 3469 | 2 | Internal parameters |
| 3470 | 1 | Internal parameters |
| 3471 | 31 | Internal parameters |
| 3472 | 31 | Internal parameters |
| 3473 | 31 | Internal parameters |
| 3474 | 31 | Internal parameters |
| 3475 | 31 | Internal parameters |
| 3476 | 0 | Internal parameters |
| 3477 | 11 | Internal parameters |
| 3478 | 25 | Internal parameters |
| 3479 | 220 | Internal parameters |
| 3480 | 10 | Internal parameters |
| 3481 | 20 | Internal parameters |
| 3482 | 2 | Internal parameters |
| 3483 | 2 | Internal parameters |
| 3484 | 2 | Internal parameters |
| 3485 | 0 | Internal parameters |
| 3486 | 0 | Internal parameters |
| 3487 | 0 | Internal parameters |
| 3488 | 0 | Internal parameters |
| 3489 | 0 | Internal parameters |
| 3490 | 0 | Internal parameters |
| 3491 | 0 | Internal parameters |
| 3492 | 0 | Internal parameters |
| 3493 | 0 | Inlet and outlet water temperature and indoor coil temperature of enable (Range: 0: No Enable; 1: Enable;) |
| 3494 | 0 | Water flow correction value (Range: Different models with different values) |
| 3495 | 14 | Compressor/model code, sent to the inverter and part of register 2114 ((3495 << 8) + 3300). |
| 3496 | 0 | DC water pump fo enable (Range: 0: No Enable; 1: Enable;) |
| 3497 | 1 | Refrigerant type (Range: 0: R32/R410A; 1: R290;) |
| 3498 | 5 | Heating low ambient limit, together with 3499: limit = 3498 − 3499 − 30 °C. *(firmware v1.10)* |
| 3499 | 0 | Heating low ambient limit offset, see 3498. *(firmware v1.10)* |
| 3500 | 4 | Heating low ambient limit hysteresis (K). *(firmware v1.10)* |
| 3501 | 0 | Water pump high speed power (Range: Different models with different values) |
| 3502 | 0 | Water pump medium speed power (Range: Different models with different values) |
| 3503 | 0 | Water pump low speed power (Range: Different models with different values) |
| 3504 | 0 | DC Fan high speed power (Range: Different models with different values) |
| 3505 | 0 | DC Fan medium speed power (Range: Different models with different values) |
| 3506 | 0 | DC Fan low speed power (Range: Different models with different values) |
| 3507 | 0 | Heating outlet temperature difference correction value (Range: Different models with different values) |
| 3508 | 0 | cooling outlet temperature difference correction value (Range: Different models with different values) |
| 3509 | 164 | Checksum low bit (Range: Different models with different values) |
| 3510 | 182 | Checksum high bit (Range: Different models with different values) |
