#! /bin/bash
declare -A List_acc
List_acc=([0.1]=1.11 [0.2]=1.25 [0.3]=1.43 [0.4]=1.67 [0.5]=2.0 [0.6]=2.5 [0.7]=3.33 [0.8]=5.0 [0.9]=7.51 [1.0]=7.51)
new_coeff=0.8
printf '${List_acc[%s]}=%f\n' "0.8" "${List_acc[$new_coeff]}"
