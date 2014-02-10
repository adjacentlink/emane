harness_dir=$(top_srcdir)/test/harness/profilescenario
harness_cmd=profilescenario
harness_exe=$(harness_dir)/$(harness_cmd)
harness_bin_deps=$(harness_exe) $(top_srcdir)/src/libemane/.libs/libemane.so
harness_schema=profilescenario.xsd

testcase_inputs=     \
 antenna30dsector.xml \
 antennaomni.xml      \
 antennaprofiles.xml  \
 blockageaft.xml      \
 profilescenario.xml     

EXTRA_DIST=           \
 $(testcase_inputs)   \
 testcase-target.txt

all-local: testcase-output.txt
	@diff -q testcase-output.txt testcase-target.txt &> /dev/null

testcase-output.txt: $(testcase_inputs) $(harness_bin_deps)
	$(harness_exe) \
    --schema $(harness_dir)/$(harness_schema) \
    antennaprofiles.xml profilescenario.xml &> testcase-output.txt

clean-local:
	rm -f testcase-output.txt
