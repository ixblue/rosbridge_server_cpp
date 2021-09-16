#!groovy

def DOCKER_IMAGE = "docker.robopec.com/ros-melodic-mdt-build"
def PACKAGE_NAME = "p_rosbridge_server_cpp"

pipeline {
    agent { label 'x64' }
    stages {
        stage('Prepare WS') {
            steps {
                // Clean up
                sh 'rm -rf *'
                dir("ws/src/${PACKAGE_NAME}") {
                    checkout scm
                    sh 'git submodule update --init'
                }
                dir('ws/src') {
                    sh """
                        cp ${PACKAGE_NAME}/rosinstall .rosinstall
                        wstool update -j `nproc --ignore=2` || true
                    """
                }

                // Install latest version of rosunit to handle p_rosbridge_server_cpp Qt tests
                dir('tmp')
                {
                    sh 'git clone --depth=1 https://github.com/ros/ros'
                    sh 'cp -r ros/tools/rosunit ../ws/src'
                    sh 'rm -rf ros || true'
                }
            }
        }
        stage('Build') {
            agent {
                docker {
                    image "${DOCKER_IMAGE}"
                    reuseNode true
                }
            }
            steps {
                dir('ws'){
                    ansiColor('xterm') {
                        sh '''
                            . /opt/ros/$ROS_DISTRO/setup.sh
                            catkin init
                            catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-Wall' -DDISABLE_GUI:BOOL=ON -DCATKIN_ENABLE_TESTING=1
                            catkin build --no-status -j `nproc --ignore=2`
                        '''
                    }
                }
            }
        }
        stage('Test') {
            agent {
                docker {
                    image "${DOCKER_IMAGE}"
                    reuseNode true
                }
            }
            steps{
                dir("ws/src/${PACKAGE_NAME}") {
                    ansiColor('xterm') {
                        sh '''
                            . /opt/ros/$ROS_DISTRO/setup.sh
                            catkin run_tests --no-deps --this -j1
                        '''
                    }
                }
                junit 'ws/**/**/test_results/**/*.xml'
            }
        }
    }
    post {
        always {
            recordIssues tool: gcc()
            script {
                currentBuild.result = currentBuild.result ?: 'SUCCESS'
                notifyBitbucket()
            }
            sh "docker rmi ${DOCKER_IMAGE} || true"
        }
        failure {
            emailext (
                subject: "FAILED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
                body: """<p><font size="8" color="red">Build Failure!</font></p>
                <p>FAILED: Job <a href='${BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a>:</p>
                <p>Check console output at "<a href='${BUILD_URL}consoleText'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a>"</p>""",
                to: "rre@robopec.com",
                recipientProviders: [[$class: 'DevelopersRecipientProvider'], [$class: 'CulpritsRecipientProvider']]
            )
        }
    }
}

