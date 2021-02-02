/**
 * 'framework-zephyr' Jenkinsfile
 *
 * Trigger:
 * Automatic triggered when a patch set is created in framework-zephyr repository.
 *
 * Job:
 * Retrieves the necessary resources and execute 'platformio run' build command for wblgt01
 * environment.
 *
 * Parameters:
 * None
 */

// Run Jenkins pipeline on Auguste slave identified by 'docker' label
node('docker') {
    def sshRepoUrl = 'ssh://jenkins@gerrit.wiifor.com:29418/embedded_os'
    def nexusRegistryUrl = 'https://nexus.wiifor.com:8090'
    def nexusUploadStagingRegistryUrl = 'https://nexus.wiifor.com:8091'
    def nexusUploadReleaseRegistryUrl = 'https://nexus.wiifor.com:8092'

    stage('Fetch embedded-os repo sources') {
        // Fetch janus repo from gerrit
        dir('janus') {
            git branch: 'master',
                credentialsId: 'jenkins',
                url: "${sshRepoUrl}/janus"
        }

        // Fetch erpc repo from gerrit
        dir('erpc') {
            git branch: 'master',
                credentialsId: 'jenkins',
                url: "${sshRepoUrl}/erpc"
        }

        // Fetch dionys-os repo from gerrit
        dir('dionys-os') {
            git branch: 'master',
                credentialsId: 'jenkins',
                url: "${sshRepoUrl}/dionys-os"
        }

        // Fetch current repository
        dir('framework-zephyr') {
            checkout scm
        }
    }

    stage('Build nrf52 firmware') {
        docker.withRegistry(nexusRegistryUrl, 'jenkins-nexus') {
            def nrf52BuildImage = docker.image('nrf52-userdefault:latest')
            nrf52BuildImage.pull()
            nrf52BuildImage.inside(
                "--entrypoint=\"\" " +
                "-e EMBEDDED_OS_PATH=${WORKSPACE} ")
            {
                // Run inside the docker image where embedded_os is mounted in ${WORKSPACE}
                dir('janus') {
                    // Check if formatting is done properly
                    def status1 = sh(
                        returnStatus: true,
                        script: './clang-format.sh'
                    )
                    if (status1 != 0) {
                        currentBuild.result = 'FAILED'
                    }

                    // Display platformio version
                    sh 'platformio --version'

                    // Clean build directory
                    def status2 = sh(
                        returnStatus: true,
                        script: 'platformio run -e wblgt01 -t clean'
                    )
                    if (status2 != 0) {
                        currentBuild.result = 'FAILED'
                    }
                    // Build nrf52 image
                    def status3 = sh(
                        returnStatus: true,
                        script: 'platformio run -e wblgt01'
                    )
                    if (status3 != 0) {
                        currentBuild.result = 'FAILED'
                    }
                }
            }
        }
    }

    dir('janus') {
        // Uploads binary to nexus
        stage("upload"){
            sh 'ls'
            nexusArtifactUploader(
                protocol: 'https',
                nexusUrl: 'nexus.wiifor.com',
                nexusVersion: 'nexus3',
                credentialsId: 'jenkins-nexus',
                repository: 'dionys-os-delivery',
                artifacts: [
                    [artifactId: 'nrf52',
                    file: '.pio/build/wblgt01/firmware.elf',
                    type: 'elf']
                ]
            )
        }
    }
}
