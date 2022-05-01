(define (problem fugitive_catcher)
	(:domain fugitive_catcher )
	(:objects
		Cell_0 - cell
		Cell_1 - cell
		Cell_10 - cell
		Cell_11 - cell
		Cell_12 - cell
		Cell_13 - cell
		Cell_14 - cell
		Cell_15 - cell
		Cell_16 - cell
		Cell_17 - cell
		Cell_18 - cell
		Cell_19 - cell
		Cell_2 - cell
		Cell_20 - cell
		Cell_21 - cell
		Cell_22 - cell
		Cell_3 - cell
		Cell_4 - cell
		Cell_5 - cell
		Cell_6 - cell
		Cell_7 - cell
		Cell_8 - cell
		Cell_9 - cell
		Gate_0 - gate
		Gate_1 - gate
		Catcher_1 - catcher
		Fugitive_1 - fugitive
	)
	(:init
		( connected Cell_0 Cell_17 )
		( connected Cell_0 Cell_21 )
		( connected Cell_0 Cell_8 )
		( connected Cell_1 Cell_3 )
		( connected Cell_1 Cell_7 )
		( connected Cell_10 Cell_12 )
		( connected Cell_10 Cell_15 )
		( connected Cell_10 Cell_9 )
		( connected Cell_11 Cell_13 )
		( connected Cell_11 Cell_16 )
		( connected Cell_11 Cell_9 )
		( connected Cell_12 Cell_10 )
		( connected Cell_12 Cell_2 )
		( connected Cell_12 Cell_5 )
		( connected Cell_12 Cell_6 )
		( connected Cell_12 Cell_9 )
		( connected Cell_13 Cell_11 )
		( connected Cell_13 Cell_14 )
		( connected Cell_13 Cell_16 )
		( connected Cell_13 Cell_18 )
		( connected Cell_13 Cell_22 )
		( connected Cell_13 Cell_9 )
		( connected Cell_14 Cell_13 )
		( connected Cell_14 Cell_15 )
		( connected Cell_14 Cell_18 )
		( connected Cell_14 Cell_22 )
		( connected Cell_15 Cell_10 )
		( connected Cell_15 Cell_14 )
		( connected Cell_16 Cell_11 )
		( connected Cell_16 Cell_13 )
		( connected Cell_16 Cell_9 )
		( connected Cell_17 Cell_0 )
		( connected Cell_17 Cell_19 )
		( connected Cell_17 Cell_22 )
		( connected Cell_17 Cell_8 )
		( connected Cell_18 Cell_13 )
		( connected Cell_18 Cell_14 )
		( connected Cell_18 Cell_22 )
		( connected Cell_19 Cell_17 )
		( connected Cell_19 Cell_22 )
		( connected Cell_2 Cell_12 )
		( connected Cell_2 Cell_4 )
		( connected Cell_2 Cell_5 )
		( connected Cell_2 Cell_9 )
		( connected Cell_20 Cell_22 )
		( connected Cell_20 Cell_7 )
		( connected Cell_21 Cell_0 )
		( connected Cell_21 Cell_8 )
		( connected Cell_22 Cell_13 )
		( connected Cell_22 Cell_14 )
		( connected Cell_22 Cell_17 )
		( connected Cell_22 Cell_18 )
		( connected Cell_22 Cell_19 )
		( connected Cell_22 Cell_20 )
		( connected Cell_3 Cell_1 )
		( connected Cell_3 Cell_7 )
		( connected Cell_4 Cell_2 )
		( connected Cell_5 Cell_12 )
		( connected Cell_5 Cell_2 )
		( connected Cell_5 Cell_9 )
		( connected Cell_6 Cell_12 )
		( connected Cell_7 Cell_1 )
		( connected Cell_7 Cell_20 )
		( connected Cell_7 Cell_3 )
		( connected Cell_8 Cell_0 )
		( connected Cell_8 Cell_17 )
		( connected Cell_8 Cell_21 )
		( connected Cell_9 Cell_10 )
		( connected Cell_9 Cell_11 )
		( connected Cell_9 Cell_12 )
		( connected Cell_9 Cell_13 )
		( connected Cell_9 Cell_16 )
		( connected Cell_9 Cell_2 )
		( connected Cell_9 Cell_5 )
		( connected Gate_0 Cell_17 )
		( connected Gate_0 Cell_18 )
		( connected Gate_0 Cell_22 )
		( connected Gate_1 Cell_1 )
		( connected Gate_1 Cell_2 )
		( connected Gate_1 Cell_3 )
		( connected Gate_1 Cell_4 )
		( is_in Catcher_1 Cell_20 )
		( is_in Fugitive_1 Cell_3 )
	)
	(:goal
			(captured Fugitive_1)

	)
)