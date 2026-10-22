	.text
	.syntax unified
	.eabi_attribute	67, "2.09"
	.cpu	cortex-m33
	.eabi_attribute	6, 17
	.eabi_attribute	7, 77
	.eabi_attribute	8, 0
	.eabi_attribute	9, 3
	.fpu	fpv5-sp-d16
	.eabi_attribute	27, 1
	.eabi_attribute	36, 1
	.eabi_attribute	46, 1
	.eabi_attribute	34, 0
	.eabi_attribute	17, 1
	.eabi_attribute	20, 1
	.eabi_attribute	21, 1
	.eabi_attribute	23, 3
	.eabi_attribute	24, 1
	.eabi_attribute	25, 1
	.eabi_attribute	28, 1
	.eabi_attribute	38, 1
	.eabi_attribute	18, 4
	.eabi_attribute	26, 2
	.eabi_attribute	14, 0
	.file	"regulator.c"
	.globl	reference_step
	.p2align	2
	.type	reference_step,%function
	.code	16
	.thumb_func
reference_step:
	.fnstart
	vldr	s0, [r0]
	vldr	s2, [r0, #20]
	vldr	s4, .LCPI0_0
	vldr	s6, .LCPI0_1
	vmul.f32	s0, s0, s4
	vmul.f32	s2, s2, s6
	vadd.f32	s0, s0, s2
	vstr	s0, [r1]
	vldr	s2, [r0, #20]
	vldr	s4, .LCPI0_2
	vsub.f32	s0, s2, s0
	vmul.f32	s0, s0, s4
	vstr	s0, [r1, #4]
	vldr	s0, [r0]
	vldr	s8, .LCPI0_3
	vldr	s2, [r0, #4]
	vldr	s10, .LCPI0_4
	vldr	s4, [r0, #8]
	vmul.f32	s0, s0, s8
	vldr	s8, .LCPI0_5
	vmul.f32	s2, s2, s10
	vldr	s6, [r0, #12]
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s4, s8
	vldr	s4, .LCPI0_6
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s6, s4
	vldr	s4, [r0, #16]
	vldr	s6, .LCPI0_7
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s4, s6
	vldr	s4, [r0, #20]
	vldr	s6, .LCPI0_8
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s4, s6
	vadd.f32	s0, s0, s2
	vstr	s0, [r1, #8]
	vldr	s0, [r0]
	vldr	s2, [r0, #20]
	vldr	s4, .LCPI0_9
	vldr	s6, .LCPI0_10
	vmul.f32	s0, s0, s4
	vmul.f32	s2, s2, s6
	vadd.f32	s0, s0, s2
	vstr	s0, [r2]
	vldr	s0, [r0]
	vldr	s2, [r0, #20]
	vldr	s4, .LCPI0_11
	vldr	s6, .LCPI0_12
	vmul.f32	s0, s0, s4
	vmul.f32	s2, s2, s6
	vadd.f32	s0, s0, s2
	vstr	s0, [r2, #4]
	vldr	s0, [r0]
	vldr	s8, .LCPI0_13
	vldr	s2, [r0, #4]
	vldr	s10, .LCPI0_14
	vldr	s4, [r0, #8]
	vmul.f32	s0, s0, s8
	vldr	s8, .LCPI0_15
	vmul.f32	s2, s2, s10
	vldr	s6, [r0, #20]
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s4, s8
	vldr	s4, .LCPI0_16
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s6, s4
	vadd.f32	s0, s0, s2
	vstr	s0, [r2, #8]
	vldr	s0, [r0]
	vldr	s8, .LCPI0_17
	vldr	s2, [r0, #4]
	vldr	s10, .LCPI0_18
	vldr	s4, [r0, #8]
	vmul.f32	s0, s0, s8
	vldr	s8, .LCPI0_19
	vmul.f32	s2, s2, s10
	vldr	s6, [r0, #12]
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s4, s8
	vldr	s4, .LCPI0_20
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s6, s4
	vldr	s6, [r0, #20]
	vldr	s8, .LCPI0_21
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s6, s8
	vadd.f32	s0, s0, s2
	vstr	s0, [r2, #12]
	vldr	s0, [r0]
	vldr	s6, .LCPI0_22
	vldr	s2, [r0, #4]
	vmul.f32	s0, s0, s6
	vldr	s6, .LCPI0_23
	vldr	s8, [r0, #8]
	vmul.f32	s2, s2, s6
	vldr	s6, .LCPI0_24
	vldr	s10, [r0, #12]
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s8, s6
	vldr	s6, .LCPI0_25
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s10, s6
	vldr	s6, [r0, #16]
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s6, s4
	vldr	s4, [r0, #20]
	vldr	s6, .LCPI0_26
	vadd.f32	s0, s0, s2
	vmul.f32	s2, s4, s6
	vadd.f32	s0, s0, s2
	vstr	s0, [r2, #16]
	ldr	r0, [r0, #20]
	str	r0, [r2, #20]
	bx	lr
	.p2align	2
.LCPI0_0:
	.long	1064027594
.LCPI0_1:
	.long	1034015168
.LCPI0_2:
	.long	1170495868
.LCPI0_3:
	.long	977253394
.LCPI0_4:
	.long	967772681
.LCPI0_5:
	.long	1025154983
.LCPI0_6:
	.long	1041515404
.LCPI0_7:
	.long	1062272795
.LCPI0_8:
	.long	912520474
.LCPI0_9:
	.long	1060830122
.LCPI0_10:
	.long	1049233576
.LCPI0_11:
	.long	1060865233
.LCPI0_12:
	.long	1049163350
.LCPI0_13:
	.long	1058497447
.LCPI0_14:
	.long	995616236
.LCPI0_15:
	.long	1049798496
.LCPI0_16:
	.long	1039369750
.LCPI0_17:
	.long	1057899001
.LCPI0_18:
	.long	998366187
.LCPI0_19:
	.long	1051750806
.LCPI0_20:
	.long	976043190
.LCPI0_21:
	.long	1036169626
.LCPI0_22:
	.long	1057077443
.LCPI0_23:
	.long	999967441
.LCPI0_24:
	.long	1053936071
.LCPI0_25:
	.long	1000246807
.LCPI0_26:
	.long	1033251503
.Lfunc_end0:
	.size	reference_step, .Lfunc_end0-reference_step
	.cantunwind
	.fnend

	.globl	deadtime_voltage
	.p2align	2
	.type	deadtime_voltage,%function
	.code	16
	.thumb_func
deadtime_voltage:
	.fnstart
	vldr	s2, .LCPI1_0
	vldr	s6, .LCPI1_1
	vdiv.f32	s0, s0, s2
	vcmpe.f32	s0, #0
	vmov.f32	s2, s0
	vneg.f32	s4, s0
	vmrs	APSR_nzcv, fpscr
	it	mi
	vmovmi.f32	s2, s4
	vcmpe.f32	s2, s6
	vmrs	APSR_nzcv, fpscr
	bge	.LBB1_2
	vldr	s4, .LCPI1_2
	movw	r1, #562
	vmul.f32	s2, s2, s4
	vcvt.u32.f32	s4, s2
	vmov	r0, s4
	cmp	r0, r1
	it	lo
	movlo	r1, r0
	movw	r0, :lower16:deadtime_table
	movt	r0, :upper16:deadtime_table
	add.w	r0, r0, r1, lsl #2
	vmov	s4, r1
	vcvt.f32.u32	s4, s4
	vldr	s6, [r0]
	vldr	s8, [r0, #4]
	vsub.f32	s2, s2, s4
	vsub.f32	s4, s8, s6
	vmul.f32	s2, s2, s4
	vadd.f32	s2, s6, s2
	b	.LBB1_3
.LBB1_2:
	vmov.f32	s2, #1.000000e+00
.LBB1_3:
	vldr	s4, .LCPI1_3
	vcmpe.f32	s0, #0
	vneg.f32	s0, s2
	vmrs	APSR_nzcv, fpscr
	it	mi
	vmovmi.f32	s2, s0
	vmul.f32	s0, s2, s4
	bx	lr
	.p2align	2
.LCPI1_0:
	.long	1036831949
.LCPI1_1:
	.long	1086355198
.LCPI1_2:
	.long	1119565476
.LCPI1_3:
	.long	1075661138
.Lfunc_end1:
	.size	deadtime_voltage, .Lfunc_end1-deadtime_voltage
	.cantunwind
	.fnend

	.globl	regulate_current
	.p2align	2
	.type	regulate_current,%function
	.code	16
	.thumb_func
regulate_current:
	.fnstart
	vldr	s4, .LCPI2_2
	vldr	s10, .LCPI2_0
	vdiv.f32	s4, s2, s4
	vldr	s12, .LCPI2_1
	vmul.f32	s2, s2, s10
	vcmpe.f32	s4, #0
	vneg.f32	s8, s4
	vmov.f32	s6, s4
	vmrs	APSR_nzcv, fpscr
	it	mi
	vmovmi.f32	s6, s8
	vldr	s8, .LCPI2_3
	vcmpe.f32	s6, s8
	vmul.f32	s8, s3, s12
	vmrs	APSR_nzcv, fpscr
	vadd.f32	s2, s2, s8
	bge	.LBB2_2
	vldr	s8, .LCPI2_4
	movw	r2, #562
	vmul.f32	s6, s6, s8
	vcvt.u32.f32	s8, s6
	vmov	r1, s8
	cmp	r1, r2
	it	lo
	movlo	r2, r1
	movw	r1, :lower16:deadtime_table
	movt	r1, :upper16:deadtime_table
	add.w	r1, r1, r2, lsl #2
	vmov	s8, r2
	vcvt.f32.u32	s8, s8
	vldr	s10, [r1]
	vldr	s12, [r1, #4]
	vsub.f32	s6, s6, s8
	vsub.f32	s8, s12, s10
	vmul.f32	s6, s6, s8
	vadd.f32	s6, s10, s6
	b	.LBB2_3
.LBB2_2:
	vmov.f32	s6, #1.000000e+00
.LBB2_3:
	vcmpe.f32	s4, #0
	vldr	s8, .LCPI2_5
	vneg.f32	s4, s6
	vmrs	APSR_nzcv, fpscr
	it	mi
	vmovmi.f32	s6, s4
	vmul.f32	s4, s6, s8
	vldr	s6, .LCPI2_6
	vadd.f32	s2, s2, s4
	vmul.f32	s4, s0, s6
	vadd.f32	s4, s4, s1
	vadd.f32	s2, s4, s2
	vmov.f32	s4, #-1.800000e+01
	vmov.f32	s6, #1.800000e+01
	vmul.f32	s8, s2, s0
	vcmpe.f32	s4, s2
	vmrs	APSR_nzcv, fpscr
	vselgt.f32	s4, s4, s2
	vcmpe.f32	s4, s6
	vmrs	APSR_nzcv, fpscr
	vcmpe.f32	s8, #0
	vselgt.f32	s4, s6, s4
	vmrs	APSR_nzcv, fpscr
	bmi	.LBB2_5
	vcmp.f32	s4, s2
	vmrs	APSR_nzcv, fpscr
	bne	.LBB2_6
.LBB2_5:
	vldr	s6, .LCPI2_7
	vldr	s8, .LCPI2_8
	vmul.f32	s0, s0, s6
	vdiv.f32	s0, s0, s8
	vadd.f32	s1, s0, s1
.LBB2_6:
	vmov.f32	s0, #2.400000e+01
	vmov.f32	s12, #1.000000e+00
	vmov.f32	s6, #5.000000e-01
	vdiv.f32	s8, s4, s0
	vldr	s4, .LCPI2_9
	vadd.f32	s10, s8, s12
	vmul.f32	s10, s10, s6
	vmul.f32	s10, s10, s4
	vcvt.u32.f32	s14, s10
	vcvt.f32.u32	s3, s14
	vsub.f32	s10, s10, s3
	vsub.f32	s8, s12, s8
	vcmpe.f32	s10, s6
	vmul.f32	s8, s8, s6
	vmrs	APSR_nzcv, fpscr
	vmov	r1, s14
	ble	.LBB2_8
	movs	r2, #1
	b	.LBB2_9
.LBB2_8:
	vcmp.f32	s10, s6
	vmrs	APSR_nzcv, fpscr
	ite	eq
	andeq	r2, r1, #1
	movne	r2, #0
.LBB2_9:
	vmul.f32	s8, s8, s4
	vcvt.u32.f32	s10, s8
	vcvt.f32.u32	s12, s10
	vsub.f32	s8, s8, s12
	vcmpe.f32	s8, s6
	add	r1, r2
	vmrs	APSR_nzcv, fpscr
	vmov	r2, s10
	ble	.LBB2_11
	movs	r3, #1
	b	.LBB2_12
.LBB2_11:
	vcmp.f32	s8, s6
	vmrs	APSR_nzcv, fpscr
	ite	eq
	andeq	r3, r2, #1
	movne	r3, #0
.LBB2_12:
	add	r2, r3
	vmov	s6, r1
	vmov	s8, r2
	vcvt.f32.u32	s6, s6
	vcvt.f32.u32	s8, s8
	vsub.f32	s6, s6, s8
	vmul.f32	s0, s6, s0
	vdiv.f32	s0, s0, s4
	vstmia	r0, {s0, s1, s2}
	bx	lr
	.p2align	2
.LCPI2_0:
	.long	1058419243
.LCPI2_1:
	.long	945312407
.LCPI2_2:
	.long	1036831949
.LCPI2_3:
	.long	1086355198
.LCPI2_4:
	.long	1119565476
.LCPI2_5:
	.long	1075661138
.LCPI2_6:
	.long	1059449561
.LCPI2_7:
	.long	1160563806
.LCPI2_8:
	.long	1184645120
.LCPI2_9:
	.long	1137180672
.Lfunc_end2:
	.size	regulate_current, .Lfunc_end2-regulate_current
	.cantunwind
	.fnend

	.type	deadtime_table,%object
	.section	.rodata,"a",%progbits
	.p2align	2
deadtime_table:
	.long	0
	.long	1009714692
	.long	1018101992
	.long	1023621706
	.long	1026485368
	.long	1029346418
	.long	1032001496
	.long	1033428438
	.long	1034853106
	.long	1036275182
	.long	1037694348
	.long	1039110291
	.long	1040355046
	.long	1041059329
	.long	1041761539
	.long	1042461526
	.long	1043159140
	.long	1043854235
	.long	1044546668
	.long	1045236296
	.long	1045922981
	.long	1046606584
	.long	1047286972
	.long	1047964014
	.long	1048606790
	.long	1048941773
	.long	1049274893
	.long	1049606091
	.long	1049935309
	.long	1050262491
	.long	1050587579
	.long	1050910522
	.long	1051231268
	.long	1051549766
	.long	1051865968
	.long	1052179828
	.long	1052491300
	.long	1052800341
	.long	1053106911
	.long	1053410969
	.long	1053712477
	.long	1054011400
	.long	1054307703
	.long	1054601355
	.long	1054892323
	.long	1055180580
	.long	1055466098
	.long	1055748852
	.long	1056028818
	.long	1056305974
	.long	1056580300
	.long	1056851778
	.long	1057042499
	.long	1057175365
	.long	1057306784
	.long	1057436749
	.long	1057565257
	.long	1057692301
	.long	1057817880
	.long	1057941989
	.long	1058064626
	.long	1058185791
	.long	1058305481
	.long	1058423698
	.long	1058540442
	.long	1058655713
	.long	1058769515
	.long	1058881848
	.long	1058992717
	.long	1059102125
	.long	1059210076
	.long	1059316575
	.long	1059421628
	.long	1059525239
	.long	1059627415
	.long	1059728164
	.long	1059827492
	.long	1059925407
	.long	1060021917
	.long	1060117030
	.long	1060210756
	.long	1060303103
	.long	1060394081
	.long	1060483700
	.long	1060571971
	.long	1060658903
	.long	1060744508
	.long	1060828796
	.long	1060911780
	.long	1060993470
	.long	1061073878
	.long	1061153017
	.long	1061230898
	.long	1061307534
	.long	1061382938
	.long	1061457122
	.long	1061530099
	.long	1061601882
	.long	1061672484
	.long	1061741918
	.long	1061810197
	.long	1061877336
	.long	1061943347
	.long	1062008244
	.long	1062072040
	.long	1062134748
	.long	1062196384
	.long	1062256959
	.long	1062316489
	.long	1062374985
	.long	1062432463
	.long	1062488935
	.long	1062544416
	.long	1062598918
	.long	1062652456
	.long	1062705042
	.long	1062756691
	.long	1062807415
	.long	1062857229
	.long	1062906144
	.long	1062954175
	.long	1063001335
	.long	1063047636
	.long	1063093092
	.long	1063137715
	.long	1063181518
	.long	1063224514
	.long	1063266715
	.long	1063308134
	.long	1063348783
	.long	1063388675
	.long	1063427821
	.long	1063466233
	.long	1063503925
	.long	1063540906
	.long	1063577189
	.long	1063612786
	.long	1063647707
	.long	1063681965
	.long	1063715570
	.long	1063748533
	.long	1063780865
	.long	1063812578
	.long	1063843681
	.long	1063874185
	.long	1063904100
	.long	1063933437
	.long	1063962206
	.long	1063990417
	.long	1064018080
	.long	1064045203
	.long	1064071798
	.long	1064097873
	.long	1064123437
	.long	1064148501
	.long	1064173072
	.long	1064197160
	.long	1064220773
	.long	1064243921
	.long	1064266611
	.long	1064288853
	.long	1064310654
	.long	1064332022
	.long	1064352966
	.long	1064373493
	.long	1064393612
	.long	1064413329
	.long	1064432652
	.long	1064451590
	.long	1064470148
	.long	1064488334
	.long	1064506156
	.long	1064523619
	.long	1064540732
	.long	1064557500
	.long	1064573930
	.long	1064590029
	.long	1064605803
	.long	1064621258
	.long	1064636401
	.long	1064651237
	.long	1064665772
	.long	1064680013
	.long	1064693964
	.long	1064707632
	.long	1064721022
	.long	1064734140
	.long	1064746990
	.long	1064759579
	.long	1064771910
	.long	1064783990
	.long	1064795824
	.long	1064807415
	.long	1064818769
	.long	1064829891
	.long	1064840784
	.long	1064851455
	.long	1064861906
	.long	1064872143
	.long	1064882170
	.long	1064891991
	.long	1064901610
	.long	1064911031
	.long	1064920258
	.long	1064929294
	.long	1064938145
	.long	1064946813
	.long	1064955302
	.long	1064963616
	.long	1064971758
	.long	1064979732
	.long	1064987542
	.long	1064995189
	.long	1065002679
	.long	1065010013
	.long	1065017195
	.long	1065024229
	.long	1065031117
	.long	1065037862
	.long	1065044467
	.long	1065050935
	.long	1065057268
	.long	1065063471
	.long	1065069544
	.long	1065075491
	.long	1065081314
	.long	1065087016
	.long	1065092600
	.long	1065098067
	.long	1065103421
	.long	1065108663
	.long	1065113796
	.long	1065118822
	.long	1065123743
	.long	1065128561
	.long	1065133279
	.long	1065137898
	.long	1065142422
	.long	1065146850
	.long	1065151186
	.long	1065155432
	.long	1065159589
	.long	1065163659
	.long	1065167644
	.long	1065171546
	.long	1065175366
	.long	1065179106
	.long	1065182768
	.long	1065186353
	.long	1065189863
	.long	1065193300
	.long	1065196665
	.long	1065199959
	.long	1065203184
	.long	1065206342
	.long	1065209433
	.long	1065212460
	.long	1065215423
	.long	1065218324
	.long	1065221165
	.long	1065223945
	.long	1065226668
	.long	1065229333
	.long	1065231942
	.long	1065234497
	.long	1065236998
	.long	1065239446
	.long	1065241843
	.long	1065244190
	.long	1065246488
	.long	1065248737
	.long	1065250939
	.long	1065253094
	.long	1065255205
	.long	1065257270
	.long	1065259293
	.long	1065261273
	.long	1065263211
	.long	1065265109
	.long	1065266967
	.long	1065268786
	.long	1065270566
	.long	1065272309
	.long	1065274015
	.long	1065275686
	.long	1065277321
	.long	1065278922
	.long	1065280489
	.long	1065282023
	.long	1065283525
	.long	1065284995
	.long	1065286434
	.long	1065287843
	.long	1065289223
	.long	1065290573
	.long	1065291895
	.long	1065293189
	.long	1065294455
	.long	1065295695
	.long	1065296909
	.long	1065298098
	.long	1065299261
	.long	1065300400
	.long	1065301514
	.long	1065302606
	.long	1065303674
	.long	1065304720
	.long	1065305744
	.long	1065306746
	.long	1065307727
	.long	1065308687
	.long	1065309627
	.long	1065310547
	.long	1065311448
	.long	1065312330
	.long	1065313193
	.long	1065314038
	.long	1065314866
	.long	1065315675
	.long	1065316468
	.long	1065317244
	.long	1065318004
	.long	1065318747
	.long	1065319475
	.long	1065320188
	.long	1065320885
	.long	1065321568
	.long	1065322237
	.long	1065322891
	.long	1065323531
	.long	1065324158
	.long	1065324772
	.long	1065325373
	.long	1065325961
	.long	1065326537
	.long	1065327100
	.long	1065327652
	.long	1065328192
	.long	1065328720
	.long	1065329238
	.long	1065329744
	.long	1065330240
	.long	1065330726
	.long	1065331201
	.long	1065331666
	.long	1065332121
	.long	1065332567
	.long	1065333003
	.long	1065333430
	.long	1065333848
	.long	1065334257
	.long	1065334658
	.long	1065335050
	.long	1065335434
	.long	1065335809
	.long	1065336177
	.long	1065336537
	.long	1065336890
	.long	1065337235
	.long	1065337572
	.long	1065337903
	.long	1065338226
	.long	1065338543
	.long	1065338853
	.long	1065339157
	.long	1065339454
	.long	1065339745
	.long	1065340029
	.long	1065340308
	.long	1065340581
	.long	1065340848
	.long	1065341109
	.long	1065341365
	.long	1065341615
	.long	1065341860
	.long	1065342100
	.long	1065342335
	.long	1065342565
	.long	1065342790
	.long	1065343011
	.long	1065343226
	.long	1065343437
	.long	1065343644
	.long	1065343846
	.long	1065344044
	.long	1065344238
	.long	1065344428
	.long	1065344614
	.long	1065344795
	.long	1065344973
	.long	1065345148
	.long	1065345318
	.long	1065345485
	.long	1065345648
	.long	1065345808
	.long	1065345965
	.long	1065346118
	.long	1065346268
	.long	1065346415
	.long	1065346559
	.long	1065346700
	.long	1065346837
	.long	1065346972
	.long	1065347104
	.long	1065347233
	.long	1065347360
	.long	1065347483
	.long	1065347605
	.long	1065347723
	.long	1065347839
	.long	1065347953
	.long	1065348064
	.long	1065348173
	.long	1065348280
	.long	1065348384
	.long	1065348486
	.long	1065348586
	.long	1065348684
	.long	1065348780
	.long	1065348874
	.long	1065348965
	.long	1065349055
	.long	1065349143
	.long	1065349229
	.long	1065349313
	.long	1065349396
	.long	1065349477
	.long	1065349556
	.long	1065349633
	.long	1065349709
	.long	1065349783
	.long	1065349856
	.long	1065349927
	.long	1065349996
	.long	1065350064
	.long	1065350131
	.long	1065350196
	.long	1065350260
	.long	1065350322
	.long	1065350383
	.long	1065350443
	.long	1065350502
	.long	1065350559
	.long	1065350615
	.long	1065350670
	.long	1065350724
	.long	1065350777
	.long	1065350828
	.long	1065350879
	.long	1065350928
	.long	1065350977
	.long	1065351024
	.long	1065351070
	.long	1065351116
	.long	1065351160
	.long	1065351204
	.long	1065351246
	.long	1065351288
	.long	1065351329
	.long	1065351368
	.long	1065351407
	.long	1065351446
	.long	1065351483
	.long	1065351520
	.long	1065351556
	.long	1065351591
	.long	1065351625
	.long	1065351659
	.long	1065351692
	.long	1065351724
	.long	1065351755
	.long	1065351786
	.long	1065351816
	.long	1065351846
	.long	1065351875
	.long	1065351903
	.long	1065351931
	.long	1065351958
	.long	1065351985
	.long	1065352011
	.long	1065352036
	.long	1065352061
	.long	1065352086
	.long	1065352110
	.long	1065352133
	.long	1065352156
	.long	1065352178
	.long	1065352200
	.long	1065352222
	.long	1065352243
	.long	1065352263
	.long	1065352283
	.long	1065352303
	.long	1065352322
	.long	1065352341
	.long	1065352360
	.long	1065352378
	.long	1065352396
	.long	1065352413
	.long	1065352430
	.long	1065352447
	.long	1065352463
	.long	1065352479
	.long	1065352494
	.long	1065352510
	.long	1065352525
	.long	1065352539
	.long	1065352553
	.long	1065352567
	.long	1065352581
	.long	1065352595
	.long	1065352608
	.long	1065352621
	.long	1065352633
	.long	1065352645
	.long	1065352658
	.long	1065352669
	.long	1065352681
	.long	1065352692
	.long	1065352703
	.long	1065352714
	.long	1065352725
	.long	1065352735
	.long	1065352745
	.long	1065352755
	.long	1065352765
	.long	1065352775
	.long	1065352784
	.long	1065352793
	.long	1065352802
	.long	1065352811
	.long	1065352819
	.long	1065352828
	.long	1065352836
	.long	1065352844
	.long	1065352852
	.long	1065352859
	.long	1065352867
	.long	1065352874
	.long	1065352882
	.long	1065352889
	.long	1065352896
	.long	1065352902
	.long	1065352909
	.long	1065352915
	.long	1065352922
	.long	1065352928
	.long	1065352934
	.long	1065352940
	.long	1065352946
	.long	1065352952
	.long	1065352957
	.long	1065352963
	.long	1065352968
	.long	1065352973
	.long	1065352978
	.long	1065352983
	.long	1065352988
	.long	1065352993
	.long	1065352998
	.long	1065353002
	.long	1065353007
	.long	1065353011
	.long	1065353016
	.size	deadtime_table, 2256


	.ident	"Android (6454773 based on r365631c2) clang version 9.0.8 (https://android.googlesource.com/toolchain/llvm-project 98c855489587874b2a325e7a516b99d838599c6f) (based on LLVM 9.0.8svn)"
	.section	".note.GNU-stack","",%progbits
	.addrsig
	.eabi_attribute	30, 1
